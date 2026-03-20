# Base image: Ubuntu 22.04 with ROS2 Humble installed
FROM ros:humble

# Install dependencies from apt
RUN apt-get update && apt-get install -y \
  # To use git in the container
  git \
  # pip is a package manager for Python
  python3-pip \
  # To let us sync with GitHub over SSH
  ssh-client \
  ros-humble-rosbridge-server \
  ros-humble-nav2-simple-commander \
  # TF quaternion utilities
  ros-humble-tf-transformations \
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
  && rm -rf /var/lib/apt/lists/*

# Install moteus python library
RUN pip3 install moteus

RUN pip3 install pyusb

# Inverse kinematics dependencies
RUN pip3 install \
  python-can \
  roboticstoolbox-python \
  spatialmath-python \
  spatialgeometry \
  websockets \
  swift-sim

# Install OpenCV for python (note: you already have python3-opencv via apt)
RUN pip3 install opencv-python

# Update pydocstyle to remove a deprecation warning when testing for PEP257
RUN pip install --upgrade pydocstyle

# Add a user so we can remote into this container with a non-root user
RUN useradd trickfire \
  --shell /bin/bash \
  --create-home \
  --no-log-init \
  -G video

# Copy all the bash customizations over to the user.
COPY .devcontainer/trickfire.bashrc /home/trickfire/.bashrc