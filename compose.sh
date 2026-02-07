#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Sets up a quick CoUGARs development environment
#
# Usage:
#   ./compose.sh [down|up|command]
#
# Arguments:
#   down: Stop the cougars-ct container
#   up: Start the cougars-ct container (default)
#   command: Any other arguments will be passed to the container (e.g. bash)

set -e

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source "$SCRIPT_DIR/scripts/common.sh"

PROFILES=""
arch=$(uname -m)
if [[ "$arch" == "x86_64" ]]; then
    PROFILES="--profile mapproxy"
fi

case $1 in
    "down")
        if [[ "$arch" == "x86_64" ]]; then
            printWarning "Stopping the mapproxy-ct container..."
        fi
        printWarning "Stopping the cougars-ct container..."
        docker compose -f "$SCRIPT_DIR/docker/docker-compose.yaml" $PROFILES down
        ;;
    "up" | "")
        # Allow container to forward graphical displays to host
        xhost +

        if [[ "$arch" == "x86_64" ]]; then
            printInfo "Loading the mapproxy-ct container..."
        fi
        printInfo "Loading the cougars-ct container..."
        docker compose -f "$SCRIPT_DIR/docker/docker-compose.yaml" $PROFILES up -d

        # Wait for './entrypoint.sh' to finish
        while [ "$(docker exec cougars-ct test -f /tmp/ready \
            && echo 'yes' || echo 'no')" != "yes" ]; do sleep 1; done

        # Check if a 'coug_dev' tmux session already exists
        if [ "$(docker exec -it --user frostlab-docker cougars-ct \
            tmux list-sessions | grep coug_dev)" == "" ]; then

            # If not, create a new 'coug_dev' tmux session
            printWarning "Creating a new 'coug_dev' tmux session..."
            docker exec -it --user frostlab-docker cougars-ct \
                tmuxp load -d /home/frostlab-docker/.tmuxp/coug_dev.yaml
        fi

        # Attach to the 'coug_dev' tmux session
        printInfo "Attaching to the 'coug_dev' tmux session..."
        docker exec -it --user frostlab-docker cougars-ct \
            tmux attach -t coug_dev
        ;;
    *)
        # Pass the command to the container
        docker exec -it --user frostlab-docker cougars-ct "$@"
        ;;
esac
