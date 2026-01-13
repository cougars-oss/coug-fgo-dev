#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Sets up a quick CoUGARs development environment

set -e

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

PROFILES=""
arch=$(uname -m)
if [[ "$arch" == "amd64" ]]; then
	PROFILES="--profile mapproxy"
fi

case $1 in
	"down")
		printWarning "Stopping the cougars-ct container..."
		docker compose -f $(dirname "$(readlink -f "$0")")/docker/docker-compose.yaml $PROFILES down
		;;
    *)
		# Allow container to forward graphical displays to host
		xhost +

		# Export host UID and GID for permission fixes
		export HOST_UID=$(id -u)
		export HOST_GID=$(id -g)

		printInfo "Loading the cougars-ct container..."
		docker compose -f $(dirname "$(readlink -f "$0")")/docker/docker-compose.yaml $PROFILES up -d

		# Wait for './entrypoint.sh' to finish
		while [ "$(docker exec cougars-ct ps -p 1 -o uid= | tr -d ' ')" != "$HOST_UID" ]; do sleep 1; done

		# Install vcs-defined external packages
		docker exec -it --user frostlab-docker -w /home/frostlab-docker/coug_ws/src cougars-ct \
			bash -c "vcs import . < cougars.repos"
		docker exec -it --user frostlab-docker -w /home/frostlab-docker/coug_ws/src cougars-ct \
			vcs custom --git --args submodule update --init --recursive

		# Check if a 'coug_dev' tmux session already exists
		if [ "$(docker exec -it --user frostlab-docker -e HOME=/home/frostlab-docker cougars-ct \
			tmux list-sessions | grep coug_dev)" == "" ]; then

			# If not, create a new 'coug_dev' tmux session
			printWarning "Creating a new 'coug_dev' tmux session..."
			docker exec -it --user frostlab-docker -e HOME=/home/frostlab-docker cougars-ct \
				tmux new-session -d -s coug_dev -n main -c "~"
			docker exec -it --user frostlab-docker -e HOME=/home/frostlab-docker cougars-ct \
				tmux send-keys -t coug_dev:main.0 "clear && cat /introduction.txt" C-m
		fi
		# Attach to the 'coug_dev' tmux session
		docker exec -it --user frostlab-docker -e HOME=/home/frostlab-docker cougars-ct tmux attach -t coug_dev
    ;;
esac
