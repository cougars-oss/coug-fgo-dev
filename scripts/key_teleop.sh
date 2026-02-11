#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Drives the BlueROV2 using the keyboard

script_dir="$(dirname "$(readlink -f "$0")")"
source "$script_dir/utils/common.sh"
source "$script_dir/../coug_ws/install/setup.bash"

print_info "Starting BlueROV2 keyboard teleop..."
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/blue0sim/cmd_vel
