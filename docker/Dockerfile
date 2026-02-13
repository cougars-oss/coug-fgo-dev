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

# Used to skip building architecture-incompatible packages
ENV ROS_BUILD_ARCH=$TARGETARCH
ENV RCUTILS_COLORIZED_OUTPUT=1

# Install ROS packages
USER root
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
USER frostlab-docker
ENV PATH="$PATH:/home/frostlab-docker/.local/bin"
RUN --mount=type=cache,target=/home/frostlab-docker/.cache/pip,uid=1000,gid=1000 \
    pip3 install --upgrade \
    # External repo management tool - Nelson Durrant, Jan 2026
    vcstool \
    # Terminal multiplexer - Nelson Durrant, Jan 2026
    tmuxp asciimatics Pillow

# Install tmux plugins
RUN git clone https://github.com/tmux-plugins/tpm /home/frostlab-docker/.tmux/plugins/tpm
COPY --chown=frostlab-docker:frostlab-docker docker/.tmux.conf /home/frostlab-docker/.tmux.conf
RUN /home/frostlab-docker/.tmux/plugins/tpm/bin/install_plugins

USER root
RUN mkdir -p /home/frostlab-docker/coug_ws/src \
    && chown -R frostlab-docker:frostlab-docker /home/frostlab-docker/coug_ws
WORKDIR /home/frostlab-docker/coug_ws/src
COPY --from=cacher --chown=frostlab-docker:frostlab-docker /tmp/packages .

# Install ROS package dependencies (rosdep)
RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/frostlab-docker/.ros/rosdep,uid=1000,gid=1000 \
    export GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no" \
    && vcs import . < cougars.repos \
    && vcs custom --git --args submodule update --init --recursive \
    && chown -R frostlab-docker:frostlab-docker . \
    && apt update \
    && gosu frostlab-docker rosdep update \
    && gosu frostlab-docker rosdep install --from-paths . --ignore-src -y

# Handle Docker CI triggers
ARG TRIGGER_REPO=""
ARG CACHE_BUST=1
RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/home/frostlab-docker/.ros/rosdep,uid=1000,gid=1000 \
    if [ -n "$TRIGGER_REPO" ]; then \
    export GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no" \
    && echo "TRIGGER_REPO: $TRIGGER_REPO" \
    && echo "CACHE_BUST: $CACHE_BUST" \
    && cd "${TRIGGER_REPO##*/}" \
    && git fetch --all && git reset --hard "@{u}" && git submodule update --init --recursive \
    && cd .. \
    && chown -R frostlab-docker:frostlab-docker . \
    && apt update \
    && gosu frostlab-docker rosdep update \
    && gosu frostlab-docker rosdep install --from-paths "${TRIGGER_REPO##*/}" --ignore-src -y; \
    fi \
    && rm -rf ./*

# Configure bash environment
USER frostlab-docker
RUN echo "" >> /home/frostlab-docker/.bashrc \
    && echo "# Source ROS 2 environment" >> /home/frostlab-docker/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /home/frostlab-docker/.bashrc \
    && echo "[ -f /home/frostlab-docker/coug_ws/install/setup.bash ] \
    && source /home/frostlab-docker/coug_ws/install/setup.bash" >> /home/frostlab-docker/.bashrc \
    && echo "" >> /home/frostlab-docker/.bashrc \
    && echo "# Script tab completions" >> /home/frostlab-docker/.bashrc \
    && echo "[ -f /completions.sh ] && source /completions.sh" >> /home/frostlab-docker/.bashrc \
    && touch /home/frostlab-docker/.hushlogin

USER root
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/completions.sh /completions.sh
RUN chmod +x /entrypoint.sh /completions.sh

WORKDIR /home/frostlab-docker

ENTRYPOINT ["/entrypoint.sh"]
CMD ["gosu", "frostlab-docker", "/bin/bash"]
