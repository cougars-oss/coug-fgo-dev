#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs cougars-ct'

set -e

# Fix permission errors
USERNAME=frostlab-docker
TARGET_UID=$(stat -c '%u' /home/$USERNAME/coug_ws/src)
TARGET_GID=$(stat -c '%g' /home/$USERNAME/coug_ws/src)

if [ ! -z "$TARGET_GID" ]; then
    if [ "$TARGET_GID" != "$(id -g $USERNAME)" ]; then
        echo "Changing GID of $USERNAME to $TARGET_GID..."
        groupmod -o -g "$TARGET_GID" $USERNAME
    fi
fi
if [ ! -z "$TARGET_UID" ]; then
    if [ "$TARGET_UID" != "$(id -u $USERNAME)" ]; then
        echo "Changing UID of $USERNAME to $TARGET_UID..."
        usermod -o -u "$TARGET_UID" $USERNAME
    fi
fi

# Install vcs-defined external packages
CURRENT_DIR=$(pwd)
cd /home/$USERNAME/coug_ws/src
if wget -q --spider http://github.com; then
    if [ -f "cougars.repos" ]; then
        echo "Network found. Updating vcs repositories..."
        gosu $USERNAME vcs import . < cougars.repos
        gosu $USERNAME vcs custom --git --args submodule update --init --recursive
    fi
else
    echo "No network connection. Skipping vcs repository updates."
fi
cd $CURRENT_DIR

# Source ROS environment
source /opt/ros/humble/setup.bash
if [ -f "/home/$USERNAME/coug_ws/install/setup.bash" ]; then
    source "/home/$USERNAME/coug_ws/install/setup.bash"
fi

touch /tmp/ready
exec "$@"
