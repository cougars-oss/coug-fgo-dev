#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches the CoUGARs development stack for a rosbag
#
# Usage:
#   ./scripts/bag_launch.sh <bag_name> [-c] [-d <seconds>] [-r <bag_name>]
#
# Arguments:
#   <bag_name>: Name of the rosbag to play (required)
#   -c: Launch alternative localization methods for comparison
#   -d <seconds>: Delay before starting nodes (default: 5.0)
#   -r <bag_name>: Record a rosbag to ~/bags/<bag_name>

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/common.sh"
source ~/coug_ws/install/setup.bash

if [ -z "$1" ]; then
    printError "Usage: $0 <bag_name> [-c] [-d <seconds>] [-r <bag_name>]"
    exit 1
fi

PLAY_BAG_PATH="$HOME/bags/$1"
shift

if [ ! -d "$PLAY_BAG_PATH" ]; then
    printError "Bag not found: $PLAY_BAG_PATH"
    exit 1
fi

RECORD_BAG_PATH=""
COMPARE="false"
DELAY="5.0"

while getopts ":cd:r:" opt; do
    case $opt in
        c)
            COMPARE="true"
            ;;
        d)
            DELAY="$OPTARG"
            ;;
        r)
            RECORD_BAG_PATH="$HOME/bags/$OPTARG"
            ;;
        \?)
            printError "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
    esac
done

if [ -n "$RECORD_BAG_PATH" ] && [ -d "$RECORD_BAG_PATH" ]; then
    printWarning "Bag directory already exists: $RECORD_BAG_PATH"
    read -p "Do you want to overwrite it? (y/n) " -n 1 -r
    echo 
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$RECORD_BAG_PATH"
    else
        exit 1
    fi
fi

ARGS=("compare:=$COMPARE" "play_bag_path:=$PLAY_BAG_PATH" "start_delay:=$DELAY")
if [ -n "$RECORD_BAG_PATH" ]; then
    ARGS+=("record_bag_path:=$RECORD_BAG_PATH")
fi

printInfo "Launching development stack for bag: $PLAY_BAG_PATH..."
if [ -n "$RECORD_BAG_PATH" ]; then
    printInfo "Recording to bag: $RECORD_BAG_PATH"

    TEMP_LOG_DIR=$(mktemp -d)
    ROS_LOG_DIR="$TEMP_LOG_DIR" ros2 launch coug_bringup bag.launch.py "${ARGS[@]}"
    if [ -d "$RECORD_BAG_PATH" ] && [ -d "$TEMP_LOG_DIR" ]; then
        mv "$TEMP_LOG_DIR" "$RECORD_BAG_PATH/log"
    fi

    mkdir -p "$RECORD_BAG_PATH/config"
    find -L ~/coug_ws/install -type f -path "*/config/*" -name "*.yaml" -exec cp {} "$RECORD_BAG_PATH/config/" \;
else
    ros2 launch coug_bringup bag.launch.py "${ARGS[@]}"
fi
