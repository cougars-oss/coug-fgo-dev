#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches the CoUGARs visualization stack for a rosbag
#
# Usage:
#   ./scripts/viz_launch.sh <bag_name> [-m] [-d <seconds>] [-n <namespace>]
#
# Arguments:
#   <bag_name>: Name of the rosbag to play (required)
#   -m: Launch multi-agent visualization if true
#   -d <seconds>: Start offset in seconds
#   -n <namespace>: Namespace for the AUV (e.g. auv0)

script_dir="$(dirname "$(readlink -f "$0")")"
source "$script_dir/utils/common.sh"
source "$script_dir/../coug_ws/install/setup.bash"

if [ -z "$1" ]; then
    print_error "Usage: $0 <bag_name> [-m] [-d <seconds>] [-n <namespace>]"
    exit 1
fi

play_bag_path="$HOME/bags/$1"
shift

if [ ! -d "$play_bag_path" ]; then
    print_error "Bag not found: $play_bag_path"
    exit 1
fi

delay="0.0"
namespace="coug0sim"
multiagent="false"

while getopts ":d:n:m" opt; do
    case $opt in
        d)
            delay="$OPTARG"
            ;;
        n)
            namespace="$OPTARG"
            ;;
        m)
            multiagent="true"
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

args=("play_bag_path:=$play_bag_path" "start_delay:=$delay" "auv_ns:=$namespace" "multiagent_viz:=$multiagent")

print_info "Launching visualization stack for bag: $play_bag_path..."
ros2 launch coug_bringup viz.launch.py "${args[@]}"
