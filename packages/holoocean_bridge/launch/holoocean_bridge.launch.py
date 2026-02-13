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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    auv_ns = LaunchConfiguration("auv_ns")
    main_agent = LaunchConfiguration("main_agent")

    fleet_params = os.path.join(
        os.path.expanduser("~"), "config", "fleet", "holoocean_bridge_params.yaml"
    )
    auv_params = PythonExpression(
        [
            "os.path.join(os.path.expanduser('~'), 'config', '",
            auv_ns,
            "' + '_params.yaml')",
        ]
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

    # modem_link_frame = PythonExpression(
    #     [
    #         "'",
    #         auv_ns,
    #         "/modem_link' if '",
    #         auv_ns,
    #         "' != '' else 'modem_link'",
    #     ]
    # )

    dvl_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/dvl_link' if '",
            auv_ns,
            "' != '' else 'dvl_link'",
        ]
    )

    truth_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/truth_link' if '",
            auv_ns,
            "' != '' else 'truth_link'",
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
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
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
                    fleet_params,
                    auv_params,
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
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time, "gps_frame": com_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="cmd_vel_converter",
                name="cmd_vel_converter_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time, "agent_name": agent_name},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="dvl_converter",
                name="dvl_converter_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time, "dvl_frame": dvl_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="ahrs_converter",
                name="ahrs_converter_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time, "ahrs_frame": imu_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="fin_state_publisher",
                name="fin_state_publisher_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="truth_converter",
                name="truth_converter_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {
                        "use_sim_time": use_sim_time,
                        # Fix for HoloOcean offset bug
                        "com_frame": truth_link_frame,
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
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time, "agent_name": agent_name},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="imu_converter",
                name="imu_converter_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time, "imu_frame": imu_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="mag_converter",
                name="mag_converter_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time, "mag_frame": imu_link_frame},
                ],
            ),
            Node(
                package="holoocean_bridge",
                executable="wrench_converter",
                name="wrench_converter_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {
                        "use_sim_time": use_sim_time,
                        "wrench_frame": com_link_frame,
                    },
                ],
            ),
            # Set this to the starting XY position of the main AUV in HoloOcean
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_holoocean_transform",
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
