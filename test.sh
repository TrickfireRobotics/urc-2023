#!/usr/bin/env bash

colcon test \
    --base-paths /home/trickfire/NasaRmc2022

colcon test-result \
    --test-result-base /home/trickfire/NasaRmc2022 \
    --verbose
