# Dockerfile for ORB-SLAM3 (Hercules + AirMuseum), Ubuntu 22.04
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config ninja-build ca-certificates curl gnupg \
    python3-dev python3-pip \
    libopencv-dev libeigen3-dev libboost-all-dev libssl-dev \
    libatlas-base-dev libsuitesparse-dev \
    libgl1-mesa-dev libegl1-mesa-dev libglew-dev libepoxy-dev \
    libx11-dev libxrandr-dev libxi-dev libxinerama-dev libxcursor-dev \
    libavcodec-dev libavformat-dev libswscale-dev libavutil-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libwayland-dev libxkbcommon-dev wayland-protocols \
    libpython3-dev \
    tmux fonts-dejavu-core \
    sudo && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy pybind11

# Pangolin (using v0.8 for better compatibility)
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin && \
    cd /opt/Pangolin && git checkout v0.8 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_EXAMPLES=OFF \
             -DBUILD_PANGOLIN_PYTHON=OFF && \
    cmake --build . -j"$(nproc)" && \
    cmake --install . && ldconfig && \
    rm -rf /opt/Pangolin

# Set default user arguments which should be overwritten by docker build
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

# Create a non-root user with matching UID/GID, so files written into the
# bind-mounted workspace/datasets (see run_container.sh) come out owned by
# the host user instead of root.
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add a custom .bashrc with warm colors and fixed hostname
RUN echo 'export PS1="\\[\\e[1;31m\\]\\u@orbslam3_container\\[\\e[0m\\]:\\[\\e[1;33m\\]\\w\\[\\e[0m\\]\\$ "' > /home/${USERNAME}/.bashrc && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

# Set default user & shell
USER ${USERNAME}
WORKDIR /home/${USERNAME}/orb_slam3_ws
SHELL ["/bin/bash", "-c"]
RUN echo "export USER=${USERNAME}" >> ~/.bashrc

# Claude Code CLI, for working on this repo from inside the container
RUN echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && \
    curl -fsSL https://claude.ai/install.sh | bash

RUN echo "set -g mouse on" >> /home/${USERNAME}/.tmux.conf

CMD ["/bin/bash"]
