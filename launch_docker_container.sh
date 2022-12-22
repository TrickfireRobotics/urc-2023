#!/usr/bin/env bash
docker build -t trickfireimage -f .devcontainer/Dockerfile .
docker run --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" -it trickfireimage