#!/bin/bash
# Created by Nelson Durrant, Jan 2026
# 
# Drives the BlueROV2 using the keyboard

source $(dirname "$(readlink -f "$0")")/common.sh
source ~/coug_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/auv0/cmd_vel
