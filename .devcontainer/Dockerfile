# Layers are generally ordered by volatility so as to perserve cache.
# CUDA is not included in the image, run with NVIDIA Container Toolkit.

# Base image: Ubuntu 22.04 with ROS2 Humble installed
FROM ros:humble-ros-base

# Get the install arguments, or use the defaults.
# TODO: Change these to the version of CUDA and ZED we are using.
ARG CUDA_MAJOR=11
ARG CUDA_MINOR=8
ARG ZED_MAJOR=4
ARG ZED_MINOR=2

ARG ZED_SDK_URL="https://download.stereolabs.com/zedsdk/${ZED_MAJOR}.${ZED_MINOR}/cu${CUDA_MAJOR}/ubuntu22"

# Enable non-interactive mode.
ARG DEBIAN_FRONTEND=noninteractive

# Set locale.
RUN apt-get update && \
    apt-get install --no-install-recommends -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Set the timezone and keyboard layout.
ARG TZ="America/Los_Angeles"
RUN echo "${TZ}" > /etc/localtime && \
    echo "${TZ}" > /etc/timezone

RUN apt-get update && \
    echo "keyboard-configuration keyboard-configuration/layoutcode string us" | debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/modelcode string pc105" | debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/xkb-keymap select us" | debconf-set-selections && \
    apt-get install -y --no-install-recommends keyboard-configuration console-setup && \
    rm -rf /var/lib/apt/lists/*

# Download ZED SDK.
WORKDIR /tmp
RUN apt-get update && apt-get install -y wget && rm -rf /var/lib/apt/lists/*
RUN echo "Downloading ZED SDK from ${ZED_SDK_URL}" && \
    wget -qO ZED_SDK_Linux_Ubuntu.run ${ZED_SDK_URL} && \
    chmod +x ZED_SDK_Linux_Ubuntu.run

# Install APT dependencies.
RUN apt-get update && apt-get install -y \
    zstd \
    # To use git in the container
    git \
    # pip is a package manager for Python
    python3-pip \
    # To let us sync with GitHub over SSH
    ssh-client \
    ros-humble-rosbridge-server \
    # To use rosdep for installing system dependencies
    python3-rosdep \
    python3-colcon-common-extensions \
    # Install usb utilities
    usbutils \
    can-utils \
    iproute2 \
    kmod \
    # Used for cv_bridge
    python3-numpy \
    libboost-python-dev \
    # OpenCV
    libopencv-dev \
    python3-opencv \
    # Clean out the apt lists after `apt-get update`
    && rm -rf /var/lib/apt/lists/*

# Install moteus python library. 
# https://github.com/mjbots/moteus/tree/main/lib/python
RUN pip3 install moteus

# Install additional Python dependencies.
RUN pip3 install pyusb opencv-python

# Update pydocstyle to remove a deprecation warning when testing for PEP257.
RUN pip3 install --upgrade pydocstyle

# Install the ZED SDK.
RUN apt-get update && \
    ./ZED_SDK_Linux_Ubuntu.run silent && \
    ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
    rm ZED_SDK_Linux_Ubuntu.run && \
    rm -rf /var/lib/apt/lists/*

# Add a user so we can remote into this container with a non-root user
RUN useradd trickfire \
    # Bash will be its default shell
    --shell /bin/bash \
    # Give it a directory in /home
    --create-home \
    # Don't make a giant log file for login data, we don't care about it
    --no-log-init \
    # Allow the user to access video cameras.
    -G video,zed

# Copy all the bash customizations over to the user.
COPY .devcontainer/trickfire.bashrc /home/trickfire/.bashrc

# Give sudo access.
RUN echo "trickfire ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/user

# Fix permissions.
RUN rosdep fix-permissions && \
    chown -R trickfire:trickfire /usr/local/lib/python3.10/dist-packages && \
    chmod -R a+rX /usr/local/lib/python3.10/dist-packages

# Change to the non-root user.
USER trickfire

# TODO: We might want another stage to minimize the image on the rover.