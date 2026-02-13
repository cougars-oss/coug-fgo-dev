#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs cougars-ct'

set -e

# Fix permission errors
DOCKER_USER=${DOCKER_USER}
target_uid=$(stat -c '%u' /home/$DOCKER_USER/ros2_ws/src)
target_gid=$(stat -c '%g' /home/$DOCKER_USER/ros2_ws/src)

if [ ! -z "$target_gid" ]; then
    if [ "$target_gid" != "$(id -g $DOCKER_USER)" ]; then
        echo "Changing GID of $DOCKER_USER to $target_gid..."
        groupmod -o -g "$target_gid" $DOCKER_USER
    fi
fi
if [ ! -z "$target_uid" ]; then
    if [ "$target_uid" != "$(id -u $DOCKER_USER)" ]; then
        echo "Changing UID of $DOCKER_USER to $target_uid..."
        usermod -o -u "$target_uid" $DOCKER_USER
    fi
fi

# Install vcs-defined external packages
current_dir=$(pwd)
cd /home/$DOCKER_USER/ros2_ws/src
if wget -q --spider http://github.com; then
    echo "Network found. Updating vcs repositories..."
    export GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no"
    vcs import . < cougars.repos
    vcs custom --git --args submodule update --init --recursive
    chown -R $DOCKER_USER:$DOCKER_USER .
else
    echo "No network connection. Skipping vcs repository updates."
fi
cd $current_dir

# Start SSH server
if [ -x /usr/sbin/sshd ]; then
    echo "Starting SSH server on port 2222..."
    sudo /usr/sbin/sshd
fi

touch /tmp/ready
exec "$@"
