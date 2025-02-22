# Base image: Ubuntu 20.04 with ROS2 Humble installed
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
  # Install usb utilities
  usbutils \ 
  can-utils \
  iproute2 \
  kmod \ 
  # Clean out the apt lists after `apt-get update`
  && rm -rf /var/lib/apt/lists/*


#Install moteus python library https://github.com/mjbots/moteus/tree/main/lib/python
RUN pip3 install moteus

RUN pip3 install pyusb

RUN pip3 install scipy==1.11.0
# Install inverse kinematics library https://github.com/Phylliade/ikpy 
RUN pip3 install roboticstoolbox-python

# Update pydocstyle to remove a deprecation warning when testing for PEP257
RUN pip install --upgrade pydocstyle

# Add a user so we can remote into this container with a non-root user
RUN useradd trickfire \
  # Bash will be its default shell
  --shell /bin/bash \
  # Give it a directory in /home
  --create-home \
  # Don't make a giant log file for login data, we don't care about it
  --no-log-init

# Copy all the bash customizations over to the user.
COPY .devcontainer/trickfire.bashrc /home/trickfire/.bashrc
