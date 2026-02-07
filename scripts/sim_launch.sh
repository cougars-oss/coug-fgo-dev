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

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/common.sh"
source ~/coug_ws/install/setup.bash

URDF="urdf/couguv_holoocean.urdf.xacro"
AGENTS=1
BAG_PATH=""
COMPARE="false"

while getopts ":bcmr:" opt; do
    case $opt in
        b)
            URDF="urdf/bluerov2_holoocean/bluerov2_holoocean.urdf.xacro"
            ;;
        c)
            COMPARE="true"
            ;;
        m)
            AGENTS=3
            ;;
        r)
            if [[ "$OPTARG" == -* ]]; then
                printError "Option -r requires an argument." >&2
                exit 1
            fi
            BAG_PATH="$HOME/bags/$OPTARG"
            ;;
        \?)
            printError "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            printError "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

if [ -n "$BAG_PATH" ] && [ -d "$BAG_PATH" ]; then
    printWarning "Bag directory already exists: $BAG_PATH"
    read -p "Do you want to overwrite it? (y/n) " -n 1 -r
    echo 
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$BAG_PATH"
    else
        exit 1
    fi
fi

ARGS=("urdf_file:=$URDF" "num_agents:=$AGENTS" "compare:=$COMPARE")
if [ -n "$BAG_PATH" ]; then
    ARGS+=("bag_path:=$BAG_PATH")
fi

printInfo "Launching simulation stack..."
if [ -n "$BAG_PATH" ]; then
    printInfo "Recording to bag: $BAG_PATH"

    TEMP_LOG_DIR=$(mktemp -d)
    ROS_LOG_DIR="$TEMP_LOG_DIR" ros2 launch coug_bringup sim.launch.py "${ARGS[@]}"
    if [ -d "$BAG_PATH" ] && [ -d "$TEMP_LOG_DIR" ]; then
        mv "$TEMP_LOG_DIR" "$BAG_PATH/log"
    fi

    mkdir -p "$BAG_PATH/config"
    find -L ~/coug_ws/install -type f \( -path "*/config/*" -o -path "*/rviz/*" -o -path "*/mapviz/*" -o -path "*/plotjuggler/*" \) -exec cp {} "$BAG_PATH/config/" \;
else
    ros2 launch coug_bringup sim.launch.py "${ARGS[@]}"
fi
