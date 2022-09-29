# Base image: Ubuntu 20.04 with ROS2 Galactic installed
FROM ros:galactic

# Install dependencies from apt
RUN apt-get update && apt-get install -y \
  # To use git in the container
  git \
  # pip is a package manager for Python
  python3-pip \
  # To let us sync with GitHub over SSH
  ssh-client \
  # A command-line tool for installing system dependencies
  python3-rosdep \
  # Intel RealSense SDK
  librealsense2-utils \
  librealsense2-dev \
  # Clean out the apt lists after `apt-get update`
  && rm -rf /var/lib/apt/lists/*

# Setup rosdep
RUN rosdep init
RUN rosdep update
RUN rosdep install -y \
  # Ignores keys that are found to be catkin or ament packages
  -i \
  # Acts upon all catkin packages in 'src'
  --from-path src \
  # Set to current ROS distro
  --rosdistro $ROS_DISTRO \
  # Skips librealsense2 key
  --skipkeys=librealsense2

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
