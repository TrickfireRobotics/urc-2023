# Ubuntu 22.04 with ROS 2 Humble
FROM ros:humble AS viator

# ---- system packages ----
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-pip \
    ssh-client \
    ros-humble-rosbridge-server \
    usbutils \
    can-utils \
    iproute2 \
    kmod \
    # cv_bridge
    python3-numpy \
    libboost-python-dev \
    libopencv-dev \
    python3-opencv

RUN pip3 install --no-cache-dir \
    moteus \
    pyusb \
    pybind11 \
    pybind11-stubgen \
    "pydocstyle>=6.3.0"

# ---- dev tooling ----

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - \
    && DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y nodejs

RUN pip3 install --no-cache-dir ruff mypy pre-commit \
    && curl -sSL "https://github.com/mvdan/sh/releases/download/v3.10.0/shfmt_v3.10.0_linux_$(dpkg --print-architecture)" \
    -o /usr/local/bin/shfmt \
    && chmod +x /usr/local/bin/shfmt

# ---- myactuator_rmd (RMD-X8 motor driver) ----

RUN git clone --depth 1 https://github.com/2b-t/myactuator_rmd.git /tmp/myactuator_rmd \
    && cmake -S /tmp/myactuator_rmd -B /tmp/myactuator_rmd/build -D PYTHON_BINDINGS=on \
    && cmake --build /tmp/myactuator_rmd/build -j "$(nproc)" \
    && cmake --install /tmp/myactuator_rmd/build \
    && pip3 install /tmp/myactuator_rmd \
    && pybind11-stubgen myactuator_rmd_py -o /usr/local/lib/python3.10/dist-packages \
    && rm -rf /tmp/myactuator_rmd

# ---- user ----
RUN useradd trickfire --shell /bin/bash --create-home --no-log-init -G video,sudo \
    && echo "trickfire:trickfire" | chpasswd \
    && touch /home/trickfire/.sudo_as_admin_successful \
    && chown trickfire:trickfire /home/trickfire/.sudo_as_admin_successful

COPY .devcontainer/trickfire.bashrc /home/trickfire/.bashrc

USER trickfire
