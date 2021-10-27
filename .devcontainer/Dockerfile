# Base image: Ubuntu 20.04 with ROS2 Foxy installed
FROM ros:foxy

# Copy all the bash customizations over to the container.
COPY .devcontainer/root.bashrc /root/.bashrc

# Install dependencies from apt
RUN apt-get update && apt-get install -y \
  # To run git commands in the container
  git \
  # To let us sync with GitHub over SSH
  ssh-client \
  # Clean out the apt lists after `apt-get update`
  && rm -rf /var/lib/apt/lists/*

# Add a user so we can remote into this container with a non-root user
RUN useradd trickfire \
  # Bash will be its default shell
  --shell /bin/bash \
  # Give it a directory in /home
  --create-home \
  # Don't make a giant log file for login data, we don't care about it
  --no-log-init
