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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_file = LaunchConfiguration("urdf_file")
    auv_ns = LaunchConfiguration("auv_ns")
    set_origin = LaunchConfiguration("set_origin")
    compare = LaunchConfiguration("compare")

    coug_des_dir = get_package_share_directory("coug_description")
    coug_des_launch_dir = os.path.join(coug_des_dir, "launch")
    coug_loc_dir = get_package_share_directory("coug_localization")
    coug_loc_launch_dir = os.path.join(coug_loc_dir, "launch")
    coug_fgo_dir = get_package_share_directory("coug_fgo")
    coug_fgo_launch_dir = os.path.join(coug_fgo_dir, "launch")
    coug_nav_dir = get_package_share_directory("coug_navigation")
    coug_nav_launch_dir = os.path.join(coug_nav_dir, "launch")

    coug_des_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_des_launch_dir, "coug_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "urdf_file": urdf_file,
            "auv_ns": auv_ns,
        }.items(),
    )

    coug_loc_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_loc_launch_dir, "coug_localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
            "compare": compare,
        }.items(),
    )

    coug_fgo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
            "set_origin": set_origin,
            "compare": compare,
        }.items(),
    )

    coug_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_nav_launch_dir, "coug_navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
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
            "urdf_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("coug_description"),
                    "urdf",
                    "couguv_holoocean.urdf.xacro",
                ]
            ),
            description="URDF or Xacro file to load",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "auv_ns",
            default_value="auv0",
            description="Namespace for the AUV (e.g. auv0)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "set_origin",
            default_value="true",
            description="Whether to set the origin (true) or subscribe to it (false)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "compare",
            default_value="false",
            description="Launch additional localization nodes if true",
        )
    )

    ld.add_action(coug_des_cmd)
    ld.add_action(coug_loc_cmd)
    ld.add_action(coug_fgo_cmd)
    ld.add_action(coug_nav_cmd)

    return ld
