#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs holoocean-ct'

set -e

# Fix permission errors
USERNAME=ue4
TARGET_UID=$(stat -c '%u' /home/$USERNAME/config)
TARGET_GID=$(stat -c '%g' /home/$USERNAME/config)

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

# Source ROS environment
source /opt/ros/humble/setup.bash
if [ -f "/home/$USERNAME/ros2_ws/install/setup.bash" ]; then
    source "/home/$USERNAME/ros2_ws/install/setup.bash"
fi

touch /tmp/ready
exec "$@"
