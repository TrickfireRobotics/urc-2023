#Bind the host machine's urc-2023 root folder to the container's target path.
#Launch the "trickfireimage" image by giving shell control over to the container
#Launch the robot code
docker run --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" -it --privileged --network=host trickfireimage /home/trickfire/urc-2023/launch.sh