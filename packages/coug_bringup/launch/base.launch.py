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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    multiagent_viz = LaunchConfiguration("multiagent_viz")
    auv_ns = LaunchConfiguration("auv_ns")

    coug_mapviz_dir = get_package_share_directory("coug_mapviz")
    coug_mapviz_launch_dir = os.path.join(coug_mapviz_dir, "launch")
    coug_rviz_dir = get_package_share_directory("coug_rviz")
    coug_rviz_launch_dir = os.path.join(coug_rviz_dir, "launch")
    coug_rqt_dir = get_package_share_directory("coug_rqt")
    coug_rqt_launch_dir = os.path.join(coug_rqt_dir, "launch")

    coug_mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_mapviz_launch_dir, "coug_mapviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "multiagent_viz": multiagent_viz,
            "auv_ns": auv_ns,
        }.items(),
    )

    coug_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_rviz_launch_dir, "coug_rviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "multiagent_viz": multiagent_viz,
            "auv_ns": auv_ns,
        }.items(),
    )

    coug_rqt_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_rqt_launch_dir, "coug_rqt.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "multiagent_viz": multiagent_viz,
            "auv_ns": auv_ns,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation/rosbag clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "multiagent_viz",
            default_value="false",
            description="Use multi-agent visualization config if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "auv_ns",
            default_value="auv0",
            description="Namespace for the AUV (e.g. auv0)",
        ),
    )
    ld.add_action(coug_mapviz_cmd)
    ld.add_action(coug_rviz_cmd)
    ld.add_action(coug_rqt_cmd)

    return ld
