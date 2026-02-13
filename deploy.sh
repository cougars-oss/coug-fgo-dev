#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Deploys the specified tmux session in the cougars-ct container
#
# Usage:
#   ./deploy.sh [dev|base]
#
# Arguments:
#   dev: Attach to the local 'coug_dev' tmux session (default)
#   base: Attach to the local 'coug_base' tmux session

set -e

script_dir="$(dirname "$(readlink -f "$0")")"
source "$script_dir/scripts/utils/common.sh"

profiles=""
arch=$(uname -m)
if [[ "$arch" == "x86_64" ]]; then
    profiles="--profile mapproxy"
fi

if [ -z "$1" ]; then
    session="dev"
else
    session="$1"
fi

case $session in
    "dev" | "base")
        xhost +local:root

        if [[ "$arch" == "x86_64" ]]; then
            print_info "Loading the mapproxy-ct container..."
        fi
        print_info "Loading the cougars-ct container..."
        docker compose -f "$script_dir/docker/docker-compose.yaml" $profiles up -d

        # Wait for './entrypoint.sh' to finish
        while [ "$(docker exec cougars-ct test -f /tmp/ready \
            && echo 'yes' || echo 'no')" != "yes" ]; do sleep 1; done

        # Check if a 'coug_$session' tmux session already exists
        if ! docker exec --user frostlab-docker cougars-ct \
            tmux has-session -t coug_$session 2>/dev/null; then

            # If not, create a new 'coug_$session' tmux session
            print_warning "Creating a new 'coug_$session' tmux session..."
            docker exec -it --user frostlab-docker cougars-ct \
                tmuxp load -d /home/frostlab-docker/.tmuxp/coug_$session.yaml
        fi

        # Attach to the 'coug_$session' tmux session
        print_info "Attaching to the 'coug_$session' tmux session..."
        docker exec -it --user frostlab-docker cougars-ct \
            tmux attach -t coug_$session
        ;;
    *)
        print_error "TODO: Add remote connection logic"
        exit 1
        ;;
esac
