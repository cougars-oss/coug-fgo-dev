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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_file = LaunchConfiguration("urdf_file")
    num_agents = LaunchConfiguration("num_agents")
    bag_path = LaunchConfiguration("bag_path")
    compare = LaunchConfiguration("compare")
    auv_ns = LaunchConfiguration("auv_ns")

    num_agents_int = int(context.perform_substitution(num_agents))
    bag_path_str = context.perform_substitution(bag_path)

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")
    holo_bridge_dir = get_package_share_directory("holoocean_bridge")
    holo_bridge_launch_dir = os.path.join(holo_bridge_dir, "launch")

    actions = []

    if bag_path_str:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-a",
                    "-o",
                    bag_path_str,
                    "--storage",
                    "mcap",
                ],
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coug_bringup_launch_dir, "base.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "multiagent_viz": "true" if num_agents_int > 1 else "false",
                "auv_ns": auv_ns,
            }.items(),
        )
    )

    for i in range(num_agents_int):
        if num_agents_int > 1:
            multi_auv_ns = f"coug{i}sim"
        else:
            multi_auv_ns = auv_ns

        auv_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coug_bringup_launch_dir, "auv.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "urdf_file": urdf_file,
                "auv_ns": multi_auv_ns,
                "set_origin": "true" if i == 0 else "false",
                "compare": compare,
            }.items(),
        )

        bridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(holo_bridge_launch_dir, "holoocean_bridge.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "auv_ns": multi_auv_ns,
                "main_agent": "true" if i == 0 else "false",
            }.items(),
        )

        agent_actions = [
            PushRosNamespace(multi_auv_ns),
            auv_launch,
            bridge_launch,
        ]

        actions.append(GroupAction(actions=agent_actions))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("coug_description"),
                        "urdf",
                        "couguv_holoocean.urdf.xacro",
                    ]
                ),
                description="URDF or Xacro file to load",
            ),
            DeclareLaunchArgument(
                "num_agents",
                default_value="1",
                description="Number of AUV agents to spawn",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Path to record rosbag (if empty, no recording)",
            ),
            DeclareLaunchArgument(
                "compare",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
