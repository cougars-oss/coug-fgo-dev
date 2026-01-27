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

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    auv_ns = LaunchConfiguration("auv_ns", default="auv0")
    main_agent = LaunchConfiguration("main_agent", default="true")

    pkg_share = get_package_share_directory("holoocean_bridge")
    params_file = os.path.join(pkg_share, "config", "holoocean_bridge_params.yaml")

    base_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/base_link' if '",
            auv_ns,
            "' != '' else 'base_link'",
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

    imu_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/imu_link' if '",
            auv_ns,
            "' != '' else 'imu_link'",
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

    modem_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/modem_link' if '",
            auv_ns,
            "' != '' else 'modem_link'",
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

    agent_name = PythonExpression(
        ["'", auv_ns, "' if '", auv_ns, "' != '' else 'auv0'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
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
                "main_agent",
                default_value="true",
                description="Whether this agent is the main agent (publishes world transforms)",
            ),
            Node(
                package="holoocean_bridge",
                executable="depth_converter",
                name="depth_converter_node",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "depth_frame": depth_link_frame,
                        "map_frame": "map",
                    },
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="gps_converter",
                name="gps_converter_node",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "gps_frame": com_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="cmd_vel_converter",
                name="cmd_vel_converter_node",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "agent_name": agent_name},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="dvl_converter",
                name="dvl_converter_node",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "dvl_frame": dvl_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="ahrs_converter",
                name="ahrs_converter_node",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "ahrs_frame": modem_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="fin_state_publisher",
                name="fin_state_publisher_node",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="holoocean_bridge",
                executable="truth_converter",
                name="truth_converter_node",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "com_frame": com_link_frame,
                        "base_frame": base_link_frame,
                        "map_frame": "map",
                    },
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="hsd_converter",
                name="hsd_converter_node",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "agent_name": agent_name},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="imu_converter",
                name="imu_converter_node",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "imu_frame": imu_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="mag_converter",
                name="mag_converter_node",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "mag_frame": imu_link_frame},
                ],
            ),
            # Set this to the starting position of the main AUV in HoloOcean
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="holoocean_transform",
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
                    "holoocean_global_frame",
                ],
                condition=IfCondition(main_agent),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
