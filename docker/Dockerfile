# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# --- Cache Stage ---
FROM ubuntu:22.04 AS cacher

WORKDIR /tmp/packages
COPY packages .
RUN find . -type f \! -name "package.xml" \! -name "*.repos" -delete

# --- Main Stage ---
FROM snelsondurrant/cougars:gtsam

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

ARG DOCKER_USER=frostlab-docker
ENV DOCKER_USER=$DOCKER_USER

# Used to skip building architecture-incompatible packages
ENV ROS_BUILD_ARCH=$TARGETARCH
ENV RCUTILS_COLORIZED_OUTPUT=1

# Set up SSH on port 2222
RUN mkdir -p /var/run/sshd \
    && sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config \
    && mkdir -p /home/$DOCKER_USER/.ssh \
    && echo "Host *\n\tStrictHostKeyChecking no\n\tUserKnownHostsFile /dev/null" > /home/$DOCKER_USER/.ssh/config \
    && chown -R $DOCKER_USER:$DOCKER_USER /home/$DOCKER_USER/.ssh \
    && chmod 600 /home/$DOCKER_USER/.ssh/config

# Install ROS packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt update && apt install -y --no-install-recommends \
    # Experimental DDS options - Nelson Durrant, Jan 2026
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    # Drive the AUV with a joystick or keyboard - Nelson Durrant, Jan 2026
    ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy ros-${ROS_DISTRO}-teleop-twist-keyboard \
    # Better rosbag2 storage format - Nelson Durrant, Jan 2026
    ros-${ROS_DISTRO}-rosbag2-storage-mcap

# Install Python dependencies
USER $DOCKER_USER
ENV PATH="$PATH:/home/$DOCKER_USER/.local/bin"
RUN --mount=type=cache,target=/home/$DOCKER_USER/.cache/pip,uid=1000,gid=1000 \
    pip3 install --upgrade \
    # External repo management tool - Nelson Durrant, Jan 2026
    vcstool \
    # Terminal multiplexer - Nelson Durrant, Jan 2026
    tmuxp asciimatics Pillow

# Install tmux plugins
RUN git config --system --add safe.directory "*" \
    && git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
COPY --chown=$DOCKER_USER:$DOCKER_USER docker/.tmux.conf /home/$DOCKER_USER/.tmux.conf
RUN ~/.tmux/plugins/tpm/bin/install_plugins

USER root
RUN mkdir -p /home/$DOCKER_USER/ros2_ws/src \
    && chown -R $DOCKER_USER:$DOCKER_USER /home/$DOCKER_USER/ros2_ws
WORKDIR /home/$DOCKER_USER/ros2_ws/src
COPY --from=cacher --chown=$DOCKER_USER:$DOCKER_USER /tmp/packages .

# Install ROS package dependencies (rosdep)
RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/$DOCKER_USER/.ros/rosdep,uid=1000,gid=1000 \
    vcs import . < cougars.repos \
    && vcs custom --git --args submodule update --init --recursive \
    && chown -R $DOCKER_USER:$DOCKER_USER . \
    && apt update \
    && gosu $DOCKER_USER rosdep update \
    && gosu $DOCKER_USER rosdep install --from-paths . --ignore-src -y

# Handle Docker CI triggers
ARG TRIGGER_REPO=""
ARG CACHE_BUST=1
RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/$DOCKER_USER/.ros/rosdep,uid=1000,gid=1000 \
    if [ -n "$TRIGGER_REPO" ]; then \
        echo "TRIGGER_REPO: $TRIGGER_REPO" \
        && echo "CACHE_BUST: $CACHE_BUST" \
        && REPO_PATH=$(find . -maxdepth 4 -type d -name "${TRIGGER_REPO##*/}" | head -n 1) \
        && if [ -n "$REPO_PATH" ]; then \
            git -C "$REPO_PATH" fetch --all \
            && git -C "$REPO_PATH" reset --hard "@{u}" \
            && git -C "$REPO_PATH" submodule update --init --recursive \
            && chown -R $DOCKER_USER:$DOCKER_USER . \
            && apt update \
            && gosu $DOCKER_USER rosdep update \
            && gosu $DOCKER_USER rosdep install --from-paths . --ignore-src -y; \
        fi \
    fi \
    && rm -rf ./*

# Configure bash environment
USER $DOCKER_USER
RUN echo "" >> ~/.bashrc \
    && echo "# Source ROS 2 environment" >> ~/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "# Script tab completions" >> ~/.bashrc \
    && echo "[ -f /completions.sh ] && source /completions.sh" >> ~/.bashrc \
    && touch ~/.hushlogin

USER root
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/completions.sh /completions.sh
RUN chmod +x /entrypoint.sh /completions.sh

WORKDIR /home/$DOCKER_USER

ENTRYPOINT ["/entrypoint.sh"]
CMD gosu $DOCKER_USER /bin/bash
