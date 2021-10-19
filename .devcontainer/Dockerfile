FROM ros:foxy

# Copy all the bash customizations over to the container.
COPY .devcontainer/root.bashrc /root/.bashrc

RUN apt-get update && apt-get install -y \
  git \
  ssh-client \
  && rm -rf /var/lib/apt/lists/*
