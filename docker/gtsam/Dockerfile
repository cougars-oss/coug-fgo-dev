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

FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Create the user
RUN useradd -ms /bin/bash "frostlab-docker" \
    && usermod -aG sudo "frostlab-docker" \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && echo "frostlab-docker:frostlab" | chpasswd

# Install system dependencies
RUN apt update && apt install -y --no-install-recommends \
    # Install basic system utilities - Nelson Durrant, Jan 2026
    curl wget git vim nano python3-pip python3-venv gosu procps \
    # Install remote access utilities - Nelson Durrant, Jan 2026
    openssh-server mosh tmux \
    # Install graphical utilities - Nelson Durrant, Jan 2026
    x11-apps x11-utils x11-xserver-utils xauth \
    && rm -rf /var/lib/apt/lists/*

# Install GTSAM (build from source for ARM)
RUN apt update && apt install -y --no-install-recommends \
    build-essential cmake libboost-dev libtbb-dev \
    libboost-serialization-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-date-time-dev \
    libboost-timer-dev \
    libboost-chrono-dev \
    libboost-regex-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /home/frostlab-docker
RUN git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git \
    && cd gtsam \
    && mkdir build && cd build \
    && cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_WITH_TBB=ON \
    && make -j$(nproc) \
    && sudo make install \
    && sudo ldconfig \
    && cd /home/frostlab-docker \
    && rm -rf gtsam

CMD ["bash"]