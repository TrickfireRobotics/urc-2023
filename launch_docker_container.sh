#!/usr/bin/env bash

#Build the docker image with the name "trickfireimage", and define the context of this build to be  
#the urc-2023 directory while refering the Dockerfile in .devcontainer
echo "BUILDING DOCKER IMAGE"
docker build -t trickfireimage -f .devcontainer/Dockerfile . 

#Bind the host machine's urc-2023 root folder to the container's target path.
#Then launch the "trickfireimage" image by giving shell control over to the container
echo "TRYING TO RUN DOCKER IMAGE"
#docker run --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" -it trickfireimage
echo "DOCKER CONTAINER RUNNING"