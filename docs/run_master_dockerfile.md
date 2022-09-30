# Running the master Dockerfile

As the other documenations (*[install_on_windows.md](https://github.com/TrickfireRobotics/urc-2023/blob/main/docs/install_on_windows.md), [install_on_linux.md](https://github.com/TrickfireRobotics/urc-2023/blob/main/docs/install_on_linux.md)*) deal with the development container in VSCode, this will deal with running a container without VSCode on an isolated Ubuntu machine. The "isolated Ubuntu machine" being the robot computer.

This "master Dockerfile" is meant to be used on the robot computer in order to run code without the need of VSCode injecting/mounting the codebase to the container.

## Setup - Do This Only Once
1. Install [Ubuntu 20.04](https://releases.ubuntu.com/focal/) as ROS2 Foxy runs on that version of Ubuntu
1. Install the [Docker Engine](https://docs.docker.com/engine/install/ubuntu/)

## Creating a Docker Image and Docker Container
1. Download the ```urc-2023``` repo in the ```/home/(your username)/``` directory
    1. Run ```cd ~``` to get into your home user directory
    1. Run ```git clone --recurse-submodules https://github.com/TrickfireRobotics/urc-2023.git```
1. Go into the urc-2023 directory (```cd urc-2023```) and switch to any git branch you want to
1. Type ```sudo docker build -t trickfireimage .``` This is telling Docker to build an image named ```trickfireimage``` from the Dockerfile in the same directory that we are in (the urc-2023 root directory)
1. Type ```sudo docker run -it trickfireimage``` This telling docker to run the image ```trickfireimage```, and then give control of the command-line interface (CLI) to that container
1. You should find that your user CLI has changed to something similar to ```root@(some random numbers and letters)``` 
1. After this, navigate to the trickfire urc-2023 folder: ```cd /home/trickfire/urc-2023```
1. Run ```./build.sh``` to build code, and then ```./launch.sh``` to run code
1. To shutdown this container and go back to the original CLI, type ```exit```.
