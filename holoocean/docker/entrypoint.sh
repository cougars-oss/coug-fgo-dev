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

touch /tmp/ready
exec "$@"
