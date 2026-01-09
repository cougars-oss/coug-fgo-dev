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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    auv_ns = LaunchConfiguration("auv_ns", default="auv0")
    set_origin = LaunchConfiguration("set_origin", default="true")

    pkg_share = get_package_share_directory("coug_fgo")
    params_file = os.path.join(pkg_share, "config", "fgo_params.yaml")

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
            DeclareLaunchArgument(
                "set_origin",
                default_value="true",
                description="Whether to set the origin (true) or subscribe to it (false)",
            ),
            DeclareLaunchArgument(
                "compare",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            Node(
                package="coug_fgo",
                executable="factor_graph",
                name="factor_graph_node",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,  # Namespacing for multiple agents
                        "base_frame": base_frame,  # Namespacing for multiple agents
                    },
                ],
            ),
            # TURTLMap-based config for real-time comparison
            Node(
                package="coug_fgo",
                executable="factor_graph",
                name="factor_graph_node",
                namespace="tm",
                condition=IfCondition(LaunchConfiguration("compare")),
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,

                        "imu_topic": ["/", auv_ns, "/imu/data"],
                        "gps_odom_topic": ["/", auv_ns, "/odometry/gps"],
                        "depth_odom_topic": ["/", auv_ns, "/odometry/depth"],
                        "heading_topic": ["/", auv_ns, "/imu/heading"],
                        "dvl_topic": ["/", auv_ns, "/dvl/data"],
                        "publish_global_tf": False,
                        "experimental.enable_dvl_preintegration": True,
                    },
                ],
            ),
            Node(
                package="coug_fgo",
                executable="navsat_preprocessor",
                name="navsat_preprocessor_node",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "set_origin": set_origin},
                ],
            ),
            # Uncomment this if not using a local EKF to publish the odom->base_link TF
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="static_transform",
            #     arguments=["0", "0", "0", "0", "0", "0", odom_frame, base_frame],
            #     parameters=[{"use_sim_time": use_sim_time}],
            # ),
        ]
    )
