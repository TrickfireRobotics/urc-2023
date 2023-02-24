#Bind the host machine's urc-2023 root folder to the container's target path.
#Launch the "trickfireimage" image by giving shell control over to the container
docker run --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" -it --privileged trickfireimage