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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    auv_ns = LaunchConfiguration("auv_ns")

    fleet_params = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "config",
            "fleet",
            "coug_localization_params.yaml",
        ]
    )
    auv_params = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "config",
            PythonExpression(["'", auv_ns, "' + '_params.yaml'"]),
        ]
    )

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
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            DeclareLaunchArgument(
                "compare",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            # https://github.com/CCNYRoboticsLab/imu_tools/tree/humble/imu_filter_madgwick
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
                remappings=[
                    ("imu/data_raw", "imu/data_raw"),
                    ("imu/data", "imu/data"),
                ],
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                parameters=[
                    fleet_params,
                    auv_params,
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": odom_frame,
                        "base_link_frame": base_frame,
                        "world_frame": odom_frame,
                    },
                ],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                condition=IfCondition(LaunchConfiguration("compare")),
                parameters=[
                    fleet_params,
                    auv_params,
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
                condition=IfCondition(LaunchConfiguration("compare")),
                parameters=[
                    fleet_params,
                    auv_params,
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
