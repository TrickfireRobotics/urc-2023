# Running the master Dockerfile

As the other documenations (*[install_on_windows.md](https://github.com/TrickfireRobotics/urc-2023/blob/main/docs/install_on_windows.md), [install_on_linux.md](https://github.com/TrickfireRobotics/urc-2023/blob/main/docs/install_on_linux.md)*) deal with the development container in VSCode, this will deal with running a container without VSCode on an isolated Ubuntu machine.

Specifically, this "master Dockerfile" is meant to be used on the robot in order to run code without the need of VSCode injecting the codebase.

## Setup
1. Install [Ubuntu 20.04](https://releases.ubuntu.com/focal/) as ROS2 Foxy runs on that version of Ubuntu
1. Install the [Docker Engine](https://docs.docker.com/engine/install/ubuntu/)
   1. If you run an issue after running ```sudo apt-get update```, with an error along the lines of ```does not have a Release file```, we need to manually
   tell which flavor of Docker Engine we need to download for our release of Ubuntu.
        1. To get your Ubuntu version, run ```lsb_release -a```. The codename should be ```focal``` if you downloaded the correct Ubuntu version.
        1. Type ```cd /etc/apt/sources.list.d```, then open the ```docker.list``` file by typing ```sudo nano docker.list```. You will most likely be asked for a password.
        1. Change ```$(lsb_release -cs)``` to ```focal```. Then press ```ctrl + x``` to start exiting, and then press ```y``` to save the file. Then press ```enter/return``` to exit out of the file

## Creating a Docker Image and Docker Container
1. Download the ```urc-2023``` repo in the ```/home/(your username)/``` directory
    1. Run ```cd ~``` to get into your home user directory
    1. Run ```git clone --recurse-submodules https://github.com/TrickfireRobotics/urc-2023.git```
1. Go into the urc-2023 directory (```cd urc-2023```) and switch to any git branch you want to
1. Type ```docker build -t trickfireimage .``` This is telling Docker to build an image named ```trickfireimage``` in the same directory that we are in (```/home/(your username)/urc-2023```)
1. Type ```docker run -it trickfireimage```. This telling docker to run the image ```trickfireimage```, and then give control of the command-line interface (CLI) to that container
1. You should find that your user CLI has changed to something similar to ```root@(some random numbers and letters)``` 
1. After this, navigate to the trickfire urc-2023 folder: ```cd /home/trickfire/urc-2023```
1. Run ```./build.sh``` to build code, and then ```./launch.sh``` to run code
1. To shutdown this container and go back to the original CLI, type ```exit```.
