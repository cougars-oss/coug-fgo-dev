#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches FGO localization for HoloOcean development
# Use the '-b' flag to launch the BlueROV2
# Use the '-c' flag to launch comparison localization nodes
# Use the '-m' flag to launch multiple CougUVs
# Use the '-r' flag to record a rosbag

function printInfo {
    # print blue
    echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
    # print yellow
    echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
    # print red
    echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

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
            AGENTS=2
            ;;
        r)
            BAG_PATH="$HOME/scripts/plots/$OPTARG"
            if [ -d "$BAG_PATH" ]; then
                printError "Bag already exists: $BAG_PATH"
                exit 1
            fi
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

ARGS=("urdf_file:=$URDF" "num_agents:=$AGENTS" "compare:=$COMPARE")
if [ -n "$BAG_PATH" ]; then
    ARGS+=("bag_path:=$BAG_PATH")
fi

ros2 launch coug_bringup dev.launch.py "${ARGS[@]}"