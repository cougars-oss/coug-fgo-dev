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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    auv_ns = LaunchConfiguration("auv_ns")
    set_origin = LaunchConfiguration("set_origin")

    pkg_share = get_package_share_directory("coug_fgo")
    params_file = os.path.join(pkg_share, "config", "coug_fgo_params.yaml")

    odom_frame = PythonExpression(
        ["'", auv_ns, "/odom' if '", auv_ns, "' != '' else 'odom'"]
    )

    base_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/base_link' if '",
            auv_ns,
            "' != '' else 'base_link'",
        ]
    )

    gps_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/gps_link' if '",
            auv_ns,
            "' != '' else 'gps_link'",
        ]
    )

    imu_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/imu_link' if '",
            auv_ns,
            "' != '' else 'imu_link'",
        ]
    )

    dvl_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/dvl_link' if '",
            auv_ns,
            "' != '' else 'dvl_link'",
        ]
    )

    depth_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/depth_link' if '",
            auv_ns,
            "' != '' else 'depth_link'",
        ]
    )

    com_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/com_link' if '",
            auv_ns,
            "' != '' else 'com_link'",
        ]
    )

    # modem_link_frame = PythonExpression(
    #     [
    #         "'",
    #         auv_ns,
    #         "/modem_link' if '",
    #         auv_ns,
    #         "' != '' else 'modem_link'",
    #     ]
    # )

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
                        "odom_frame": odom_frame,
                        "base_frame": base_link_frame,
                        "imu.parameter_frame": imu_link_frame,
                        "dvl.parameter_frame": dvl_link_frame,
                        "depth.parameter_frame": depth_link_frame,
                        "gps.parameter_frame": gps_link_frame,
                        "mag.parameter_frame": imu_link_frame,
                        "ahrs.parameter_frame": imu_link_frame,
                        "dynamics.parameter_frame": com_link_frame,
                    },
                ],
            ),
            # TURTLMap-based FGO for real-time comparison
            Node(
                package="coug_fgo",
                executable="factor_graph",
                name="factor_graph_node_tm",
                condition=IfCondition(LaunchConfiguration("compare")),
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": odom_frame,
                        "base_frame": base_link_frame,
                        "imu.parameter_frame": imu_link_frame,
                        "dvl.parameter_frame": dvl_link_frame,
                        "depth.parameter_frame": depth_link_frame,
                        "gps.parameter_frame": gps_link_frame,
                        "mag.parameter_frame": imu_link_frame,
                        "ahrs.parameter_frame": imu_link_frame,
                        "dynamics.parameter_frame": com_link_frame,
                        "max_keyframe_rate": 4.0,
                        "global_odom_topic": "odometry/global_tm",
                        "smoothed_path_topic": "smoothed_path_tm",
                        "publish_global_tf": False,
                        "experimental.enable_dvl_preintegration": True,
                    },
                ],
            ),
            Node(
                package="coug_fgo",
                executable="origin_manager",
                name="origin_manager_node",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "set_origin": set_origin,
                        "parameter_child_frame": gps_link_frame,
                    },
                ],
            ),
            Node(
                package="coug_fgo",
                executable="dvl_a50_twist",
                name="dvl_a50_twist_node",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "parameter_frame": dvl_link_frame,
                    },
                ],
            ),
            # Published by local EKF instead
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="odom_to_base_transform",
            #     arguments=[
            #         "--x",
            #         "0",
            #         "--y",
            #         "0",
            #         "--z",
            #         "0",
            #         "--yaw",
            #         "0",
            #         "--pitch",
            #         "0",
            #         "--roll",
            #         "0",
            #         "--frame-id",
            #         odom_frame,
            #         "--child-frame-id",
            #         base_link_frame,
            #     ],
            #     parameters=[{"use_sim_time": use_sim_time}],
            # ),
        ]
    )
