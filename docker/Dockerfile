# Copyright (c) 2026 BYU FRoSt Lab
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

# ROS_BUILD_ARCH is used to skip building architecture-incompatible packages
ENV ROS_BUILD_ARCH=$TARGETARCH

# Set default RMW implementation to CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#################################
# IMPORTANT DOCUMENTATION NOTE! #
#################################

# When adding dependencies or Dockerfile commands below, please document your changes
# following this format:
#
#     # Why do we need this dependency or command? - Full Name, Month Year
#     < Add system dependencies here >
#
# https://docs.docker.com/reference/dockerfile/

# Install ROS packages
USER root
RUN apt update && apt install -y --no-install-recommends \
    # ROS 2 middleware implementations - Nelson Durrant, Jan 2026
    ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-zenoh-cpp \
    # Helpful ROS GUI tools - Nelson Durrant, Jan 2026
    ros-humble-rqt* ros-humble-plotjuggler-ros \
    # Drive the AUV with a joystick or keyboard - Nelson Durrant, Jan 2026
    ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/frostlab-docker/coug_ws/src && chown -R frostlab-docker:frostlab-docker /home/frostlab-docker/coug_ws
WORKDIR /home/frostlab-docker/coug_ws/src
COPY --chown=frostlab-docker:frostlab-docker coug_ws/src .

# Install Python dependencies
USER frostlab-docker
RUN pip3 install --no-cache-dir \
    # Tool for managing external repos - Nelson Durrant, Jan 2026
    vcstool \
    # Used by plotting scripts - Nelson Durrant, Jan 2026
    rosbags
ENV PATH="$PATH:/home/frostlab-docker/.local/bin"

# Install custom ROS package dependencies (rosdep)
RUN vcs import . < cougars.repos \
    && git submodule update --init --recursive \
    && rosdep update \
    && sudo apt update \
    && rosdep install --from-paths . --ignore-src -y \
    && sudo rm -rf /var/lib/apt/lists/* \
    && sudo rm -rf ./*

WORKDIR /home/frostlab-docker

# Set up automatic sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /home/frostlab-docker/coug_ws/install/setup.bash" >> ~/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc \
    && touch ~/.hushlogin

CMD ["bash"]