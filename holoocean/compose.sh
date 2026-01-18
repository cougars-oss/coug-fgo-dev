#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Launches the configured scenario in HoloOcean
#
# Usage:
#   ./holoocean/compose.sh [down|up] [-b] [-m]
#
# Arguments:
#   down: Stop the holoocean-ct container
#   up: Start the holoocean-ct container (default)
#   -b: Launch the BlueROV2 scenario
#   -m: Launch the multi-CougUV scenario

set -e

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/../scripts/common.sh"

case $1 in
    "down")
        printWarning "Stopping the holoocean-ct container..."
        docker compose -f "$SCRIPT_DIR/docker/docker-compose.yaml" down
        ;;
    *)
        # Allow container to forward graphical displays to host
        xhost +

        # Export host UID and GID for permission fixes
        export HOST_UID=$(id -u)
        export HOST_GID=$(id -g)

        printInfo "Loading the holoocean-ct container..."
        docker compose -f "$SCRIPT_DIR/docker/docker-compose.yaml" up -d

        PARAMS_FILE="/home/ue4/config/coug_holoocean_params.yaml"
        
        while getopts ":bm" opt; do
            case $opt in
                b)
                    PARAMS_FILE="/home/ue4/config/bluerov2_holoocean_params.yaml"
                    ;;
                m)
                    PARAMS_FILE="/home/ue4/config/multi_coug_holoocean_params.yaml"
                    ;;
                \?)
                    printError "Invalid option: -$OPTARG" >&2
                    exit 1
                    ;;
            esac
        done

        printInfo "Launching the configured scenario in HoloOcean..."
        docker exec -it --user ue4 -e HOME=/home/ue4 -e RMW_FASTRTPS_USE_QOS_FROM_XML=1 \
            -e FASTRTPS_DEFAULT_PROFILES_FILE=/home/ue4/config/fastdds.xml holoocean-ct /bin/bash -c \
            "source ~/ros2_ws/install/setup.bash && ros2 run holoocean_main holoocean_node --ros-args \
            --params-file $PARAMS_FILE"
    ;;
esac
