# Base image: Ubuntu 20.04 with ROS2 Humble installed
FROM ros:humble

# Install dependencies from apt
RUN apt-get update 
RUN apt-get install -y \
  # To use git in the container
  git \
  # pip is a package manager for Python
  python3-pip \
  # To let us sync with GitHub over SSH
  ssh-client \
  ros-humble-rosbridge-server

RUN apt-get install -y \
  # Install usb utilities
  usbutils \ 
  can-utils \
  iproute2 \
  kmod

RUN apt-get install -y \
  # Used for cv_bridge
  python3-numpy \
  libboost-python-dev

RUN apt-get install -y \
  # OpenCV
  libopencv-dev \
  python3-opencv

# Clean out the apt lists after `apt-get update`
RUN rm -rf /var/lib/apt/lists/*

#Install moteus python library https://github.com/mjbots/moteus/tree/main/lib/python
RUN pip3 install moteus

RUN pip3 install pyusb

# Pybind build capability for the RMD lib
RUN pip3 install pybind11
RUN pip3 install pybind11-stubgen

# Update pydocstyle to remove a deprecation warning when testing for PEP257
RUN pip install --upgrade pydocstyle

# Add the RMD lib
RUN git clone https://github.com/2b-t/myactuator_rmd.git \
  && cd myactuator_rmd \
  # Build instructions taken from the github repo
  && mkdir build \
  && cd build \
  && cmake .. -D PYTHON_BINDINGS=on \
  && make -j $(nproc) \
  && sudo make install \
  # Install python bindings
  && pip3 install ../

RUN pybind11-stubgen myactuator_rmd_py -o /usr/local/lib/python3.10/dist-packages

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
