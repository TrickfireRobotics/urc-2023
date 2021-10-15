FROM ros:foxy

RUN apt-get update && apt-get install -y \
  git \
  ssh-client \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc