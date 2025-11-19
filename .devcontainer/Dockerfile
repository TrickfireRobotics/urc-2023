# Base image: Ubuntu 20.04 with ROS2 Humble installed
FROM ros:humble

# 1. Install wget, gnupg2, and ca-certificates first
RUN apt-get update && apt-get install -y wget gnupg2 ca-certificates

# 2. Add the Stereolabs GPG key using the modern method
RUN wget -qO - https://download.stereolabs.com/zedsdk/4.0/apt_key.pub \
    | gpg --dearmor -o /usr/share/keyrings/stereolabs-archive-keyring.gpg

# 3. Add Stereolabs repo for ZED SDK 4.0 on Ubuntu 22 with CUDA 11.7 using signed-by
RUN echo "deb [signed-by=/usr/share/keyrings/stereolabs-archive-keyring.gpg] https://download.stereolabs.com/zedsdk/4.0/ubuntu22_cuda11.7/ ./" \
    | tee /etc/apt/sources.list.d/zed.list

# Install dependencies from apt
RUN apt-get update && apt-get install -y \
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
  libzed-sdk4.0 \
  # Clean out the apt lists after `apt-get update`
  && rm -rf /var/lib/apt/lists/*


#Install moteus python library https://github.com/mjbots/moteus/tree/main/lib/python
RUN pip3 install moteus

# Install additional Python dependencies
RUN pip3 install pyusb opencv-python

# Update pydocstyle to remove a deprecation warning when testing for PEP257
RUN pip install --upgrade pydocstyle

# Add a user so we can remote into this container with a non-root user
RUN useradd trickfire \
  # Bash will be its default shell
  --shell /bin/bash \
  # Give it a directory in /home
  --create-home \
  # Don't make a giant log file for login data, we don't care about it
  --no-log-init \
  # Allow the user to access video cameras.
  -G video

# Copy all the bash customizations over to the user.
COPY .devcontainer/trickfire.bashrc /home/trickfire/.bashrc

# ----------------------------------------
# Clone the ZED ROS2 Wrapper repository (and its submodules) into the workspace
# ----------------------------------------
  RUN mkdir -p /home/trickfire/urc-2023/src/zed-ros2-wrapper
  WORKDIR /home/trickfire/urc-2023/src/zed-ros2-wrapper
  # Either use git submodule commands or a direct recursive clone
  RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git .  

# ---- ROSDEP SETUP ----
# Initialize, update and install rosdep dependencies
RUN rosdep init && \
    rosdep update && \
    rosdep install --from-paths /home/trickfire/urc-2023/src --ignore-src -r -y