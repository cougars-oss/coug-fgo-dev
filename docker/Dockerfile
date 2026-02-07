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

FROM snelsondurrant/cougars:gtsam

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

ENV RCUTILS_COLORIZED_OUTPUT=1

# Used to skip building architecture-incompatible packages
ENV ROS_BUILD_ARCH=$TARGETARCH

# Install ROS packages
USER root
RUN apt update && apt install -y --no-install-recommends \
    # Drive the AUV with a joystick or keyboard - Nelson Durrant, Jan 2026
    ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy ros-${ROS_DISTRO}-teleop-twist-keyboard \
    # Better rosbag2 storage format - Nelson Durrant, Jan 2026
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/frostlab-docker/coug_ws/src && chown -R frostlab-docker:frostlab-docker /home/frostlab-docker/coug_ws
WORKDIR /home/frostlab-docker/coug_ws/src
COPY --chown=frostlab-docker:frostlab-docker packages .

# Install Python dependencies
USER frostlab-docker
ENV PATH="$PATH:/home/frostlab-docker/.local/bin"
RUN pip3 install --upgrade --no-cache-dir \
    vcstool rosdoc2 tmuxp asciimatics Pillow

# Install tmux plugins
RUN git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
COPY --chown=frostlab-docker:frostlab-docker docker/.tmux.conf /home/frostlab-docker/.tmux.conf
RUN ~/.tmux/plugins/tpm/bin/install_plugins

# Install custom ROS package dependencies (rosdep)
RUN vcs import . < cougars.repos \
    && vcs custom --git --args submodule update --init --recursive \
    && rosdep update \
    && sudo apt update \
    && rosdep install --from-paths . --ignore-src -y \
    && sudo rm -rf /var/lib/apt/lists/* \
    && sudo rm -rf ./*

# Configure bash environment
RUN echo "" >> /home/frostlab-docker/.bashrc \
    && echo "# Source ROS 2 environment" >> /home/frostlab-docker/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /home/frostlab-docker/.bashrc \
    && echo "[ -f /home/frostlab-docker/coug_ws/install/setup.bash ] && source /home/frostlab-docker/coug_ws/install/setup.bash" >> /home/frostlab-docker/.bashrc \
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