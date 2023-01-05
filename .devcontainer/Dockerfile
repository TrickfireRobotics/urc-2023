# Base image: Ubuntu 20.04 with ROS2 Galactic installed
FROM ros:humble

# Install dependencies from apt
RUN apt-get update && apt-get install -y \
  # To use git in the container
  git \
  # pip is a package manager for Python
  python3-pip \
  # To let us sync with GitHub over SSH
  ssh-client \
    #Stuff
  autoconf \
  libtool \
  cmake \
  mlocate \
  libtool-bin \
  ros-humble-rviz2 \
  ros-humble-joint-state-publisher \
  ros-humble-hardware-interface \
  ros-humble-controller-interface \
  ros-humble-controller-manager \
  ros-humble-realtime-tools \
  ros-humble-ros2-control-test-assets \
  libboost-all-dev \
  ros-humble-xacro \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  # Clean out the apt lists after `apt-get update`
  && rm -rf /var/lib/apt/lists/* 

 #Get an updated version of cmake - cmake 3.8
#RUN apt purge --auto-remove cmake -y \
#  && wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null \
#  && apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' \
#  && apt update \
#  && apt install cmake -y


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

#Install ros2_canopen
#RUN apt-get install autoconf -y

#RUN rosdep init
#RUN rosdep update\
#RUN rosdep install --from-paths src --ignore-src -r -y

WORKDIR urc-2023
RUN --mount=type=bind,target="." rosdep install --from-paths src/ros2_canopen --ignore-src -r -y

#RUN rosdep update


# Copy all the bash customizations over to the user.
COPY .devcontainer/trickfire.bashrc /home/trickfire/.bashrc
