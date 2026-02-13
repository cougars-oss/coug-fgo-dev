#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches the CoUGARs simulation stack
#
# Usage:
#   ./scripts/sim_launch.sh [-b] [-c] [-m] [-r <bag_name>]
#
# Arguments:
#   -b: Launch the BlueROV2 scenario
#   -c: Launch alternative localization methods for comparison
#   -m: Launch multi-agent CougUV scenario
#   -r <bag_name>: Record a rosbag to ~/bags/<bag_name>

source ~/scripts/utils/common.sh
source ~/ros2_ws/install/setup.bash

coug_share=$(ros2 pkg prefix coug_description --share)
urdf="$coug_share/urdf/couguv_holoocean.urdf.xacro"

agents=1
bag_path=""
compare="false"
namespace="coug0sim"

while getopts ":bcmr:" opt; do
    case $opt in
        b)
            urdf="$coug_share/urdf/bluerov2_holoocean/bluerov2_holoocean.urdf.xacro"
            namespace="blue0sim"
            ;;
        c)
            compare="true"
            ;;
        m)
            agents=3
            ;;
        r)
            timestamp=$(date +"_%Y-%m-%d-%H-%M-%S")
            bag_path="$HOME/bags/${OPTARG}${timestamp}"
            ;;
        \?)
            print_error "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            print_error "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

if [ -n "$bag_path" ] && [ -d "$bag_path" ]; then
    print_warning "Bag directory already exists: $bag_path"
    read -p "Do you want to overwrite it? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$bag_path"
    else
        exit 1
    fi
fi

args=("urdf_file:=$urdf" "num_agents:=$agents" "compare:=$compare" "auv_ns:=$namespace")
if [ -n "$bag_path" ]; then
    args+=("bag_path:=$bag_path")
fi

print_info "Launching simulation stack..."
if [ -n "$bag_path" ]; then
    print_info "Recording to bag: $bag_path"

    temp_log_dir=$(mktemp -d)
    ROS_LOG_DIR="$temp_log_dir" ros2 launch coug_bringup sim.launch.py "${args[@]}"
    if [ -d "$bag_path" ] && [ -d "$temp_log_dir" ]; then
        mv "$temp_log_dir" "$bag_path/log"
    fi

    mkdir -p "$bag_path/config"
    cp -r ~/config/* "$bag_path/config/"
else
    ros2 launch coug_bringup sim.launch.py "${args[@]}"
fi
