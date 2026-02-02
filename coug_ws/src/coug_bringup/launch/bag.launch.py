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
)
import signal
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


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
    sensor_bridge_dir = get_package_share_directory("sensor_bridge")

    actions = []

    record_process = None

    if record_bag_path_str:
        record_process = ExecuteProcess(
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
        actions.append(record_process)

    if play_bag_path_str:
        play_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                play_bag_path_str,
                "--clock",
                "--start-offset",
                start_delay,
                "--remap",
                "/tf:=/tf_discard",
                "/tf_static:=/tf_static_discard",
                "bluerov2/dvl/twist:=/bluerov2/dvl/twist_discard",
                "bluerov2/imu/nav_sat_fix:=/bluerov2/gps/fix",
                "bluerov2/shallow/depth_data:=/bluerov2/odometry/depth",
            ],
        )
        actions.append(play_process)

        exit_event = LogInfo(msg="Bag playback finished, no recording to kill...")
        if record_process is not None:
            exit_event = [
                LogInfo(msg="Bag playback finished, killing recording..."),
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGINT,
                        process_matcher=matches_action(record_process),
                    )
                ),
            ]

        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=play_process,
                    on_exit=exit_event,
                )
            )
        )

    actions.append(
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

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensor_bridge_dir, "launch", "sensor_bridge.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
        }.items(),
    )

    agent_actions = [
        PushRosNamespace(auv_ns),
        auv_launch,
        sensor_launch,
    ]

    actions.append(GroupAction(actions=agent_actions))

    return actions


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
                default_value="0.0",
                description=(
                    "Time in seconds to skip from the beginning of the bag file (start offset)"
                ),
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
