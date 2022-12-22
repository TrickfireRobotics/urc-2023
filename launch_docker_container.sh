#!/usr/bin/env bash
cd .devcontainer/
docker build -t trickfireimage ..
docker run --mount type=bind,source="/home/trickfire/urc-2023",target="/home/trickfire/urc-2023" -it trickfireimage