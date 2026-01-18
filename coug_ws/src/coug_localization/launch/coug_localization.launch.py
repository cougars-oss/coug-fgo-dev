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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    auv_ns = LaunchConfiguration("auv_ns", default="auv0")

    pkg_share = get_package_share_directory("coug_localization")
    params_file = os.path.join(pkg_share, "config", "coug_localization_params.yaml")

    odom_frame = PythonExpression(
        ["'", auv_ns, "/odom' if '", auv_ns, "' != '' else 'odom'"]
    )

    base_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/base_link' if '",
            auv_ns,
            "' != '' else 'base_link'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (HoloOcean) clock if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description=(
                    "Namespace for the AUV (e.g. auv0), used for namespacing topics and frames"
                ),
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": odom_frame,
                        "base_link_frame": base_frame,
                        "world_frame": odom_frame,
                    },
                ],
                remappings=[
                    ("odometry/filtered", "odometry/local_ekf")
                ],  # Different topic to avoid conflict with IEKF
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_link_frame": base_frame,
                        "world_frame": "map",
                    },
                ],
                remappings=[
                    ("odometry/filtered", "odometry/global_ekf")
                ],  # Different topic to avoid conflict with FGO
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_map",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_link_frame": base_frame,
                        "world_frame": "map",
                    },
                ],
                remappings=[
                    ("odometry/filtered", "odometry/global_ukf")
                ],  # Different topic to avoid conflict with FGO
            ),
        ]
    )
