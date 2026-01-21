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
    GroupAction,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node


def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_delay = LaunchConfiguration("start_delay")
    urdf_file = LaunchConfiguration("urdf_file")
    auv_ns = LaunchConfiguration("auv_ns")
    play_bag_path = LaunchConfiguration("play_bag_path")
    record_bag_path = LaunchConfiguration("record_bag_path")
    compare = LaunchConfiguration("compare")
    bluerov_viz = LaunchConfiguration("bluerov_viz")

    play_bag_path_str = context.perform_substitution(play_bag_path)
    record_bag_path_str = context.perform_substitution(record_bag_path)

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")

    immediate_actions = []
    delayed_actions = []

    if play_bag_path_str:
        play_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                play_bag_path_str,
                "--clock",
                "--remap",
                "/tf:=/tf_discard",
                "/tf_static:=/tf_static_discard",
            ],
        )
        immediate_actions.append(play_process)

        immediate_actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=play_process,
                    on_exit=[
                        LogInfo(msg="Bag playback finished, shutting down..."),
                        EmitEvent(event=Shutdown(reason="Bag playback finished")),
                    ],
                )
            )
        )

        # TODO: Remove this when BlueROV TF is fixed
        delayed_actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform",
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
                    "odom",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    if record_bag_path_str:
        immediate_actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-a",
                    "-o",
                    record_bag_path_str,
                    "--storage",
                    "mcap",
                ],
            )
        )

    delayed_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coug_bringup_launch_dir, "base.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "multiagent_viz": "false",
                "bluerov_viz": bluerov_viz,
            }.items(),
        )
    )

    auv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_bringup_launch_dir, "auv.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "urdf_file": urdf_file,
            "auv_ns": auv_ns,
            "set_origin": "true",
            "compare": compare,
        }.items(),
    )

    agent_actions = [
        PushRosNamespace(auv_ns),
        auv_launch,
    ]

    delayed_actions.append(GroupAction(actions=agent_actions))

    immediate_actions.append(TimerAction(period=start_delay, actions=delayed_actions))

    return immediate_actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (rosbag) clock if true",
            ),
            DeclareLaunchArgument(
                "start_delay",
                default_value="5.0",
                description="Delay in seconds before starting the processing nodes",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value="urdf/bluerov2_heavy/bluerov2_heavy.urdf.xacro",
                description="URDF or Xacro file to load",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="bluerov2",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            DeclareLaunchArgument(
                "play_bag_path",
                default_value="",
                description="Path to play rosbag from",
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value="",
                description="Path to record rosbag (if empty, no recording)",
            ),
            DeclareLaunchArgument(
                "compare",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            DeclareLaunchArgument(
                "bluerov_viz",
                default_value="true",
                description="Load BlueROV specific viz config if true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
