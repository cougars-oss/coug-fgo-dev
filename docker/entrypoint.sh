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

if [ -z "$TARGET_UID" ]; then TARGET_UID=1000; fi
if [ -z "$TARGET_GID" ]; then TARGET_GID=1000; fi

if [ "$TARGET_GID" != "$(id -g $USERNAME)" ]; then
    echo "Changing GID of $USERNAME to $TARGET_GID..."
    groupmod -o -g "$TARGET_GID" $USERNAME
fi
if [ "$TARGET_UID" != "$(id -u $USERNAME)" ]; then
    echo "Changing UID of $USERNAME to $TARGET_UID..."
    usermod -o -u "$TARGET_UID" $USERNAME
fi

# Set up environment sourcing
if ! grep -q "source /opt/ros/humble/setup.bash" /home/$USERNAME/.bashrc; then
    echo "" >> /home/$USERNAME/.bashrc
    echo "# Source ROS 2 environment" >> /home/$USERNAME/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
    echo "[ -f /home/$USERNAME/coug_ws/install/setup.bash ] && source /home/$USERNAME/coug_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
fi
if ! grep -q "source /completions.sh" /home/$USERNAME/.bashrc; then
    echo "" >> /home/$USERNAME/.bashrc
    echo "# Script tab completions" >> /home/$USERNAME/.bashrc
    echo "[ -f /completions.sh ] && source /completions.sh" >> /home/$USERNAME/.bashrc
fi
touch /home/$USERNAME/.hushlogin

touch /tmp/ready
exec "$@"
