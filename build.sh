#!/usr/bin/env bash

colcon build \
    --symlink-install \
    --base-paths /home/trickfire/NasaRmc2022 \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Debug
