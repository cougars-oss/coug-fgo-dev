#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Deploys a tmux session in a local or remote container
#
# Usage:
#   ./deploy.sh [dev|base|remote_host]
#
# Arguments:
#   dev: Attach to the local 'coug_dev' tmux session (default)
#   base: Attach to the local 'coug_base' tmux session
#   remote_host: Mosh into a remote 'coug_auv' tmux session

set -e

script_dir="$(dirname "$(readlink -f "$0")")"
source "$script_dir/scripts/utils/common.sh"

target="${1:-dev}"
profiles=""
if [[ "$(uname -m)" == "x86_64" ]]; then
    profiles="--profile mapproxy"
    print_info "Loading the mapproxy-ct container..."
fi

xhost +local:root > /dev/null

print_info "Loading the cougars-ct container..."
docker compose -f "$script_dir/docker/docker-compose.yaml" $profiles up -d

# Wait for 'entrypoint.sh' to be ready
while ! docker exec cougars-ct [ -f /tmp/ready ]; do sleep 0.5; done

case $target in
    "dev" | "base")
        session="coug_$target"
        if ! docker exec --user frostlab-docker cougars-ct tmux has-session -t "$session" 2>/dev/null; then
            print_warning "Creating new '$session' tmux session..."
            docker exec -it --user frostlab-docker cougars-ct \
                /home/frostlab-docker/.local/bin/tmuxp load -d "/home/frostlab-docker/.tmuxp/$session.yaml"
        fi

        print_info "Attaching to '$session'..."
        docker exec -it --user frostlab-docker cougars-ct tmux attach -t "$session"
        ;;
    *)
        if ! docker exec --user frostlab-docker cougars-ct \
            ssh -p 2222 -o StrictHostKeyChecking=no -i /home/frostlab-docker/.ssh_internal/id_ed25519 \
            frostlab-docker@"$target" "tmux has-session -t coug_auv" 2>/dev/null; then

            print_warning "Creating a new 'coug_auv' tmux session on $target..."
            docker exec --user frostlab-docker cougars-ct \
                ssh -p 2222 -o StrictHostKeyChecking=no -i /home/frostlab-docker/.ssh_internal/id_ed25519 \
                frostlab-docker@"$target" "/home/frostlab-docker/.local/bin/tmuxp load -d coug_auv"
        fi

        print_info "Attaching to the 'coug_auv' tmux session on $target..."
        docker exec -it --user frostlab-docker cougars-ct \
            mosh --ssh="ssh -p 2222 -o StrictHostKeyChecking=no -i /home/frostlab-docker/.ssh_internal/id_ed25519" \
            frostlab-docker@"$target" -- tmux attach -t coug_auv
        ;;
esac
