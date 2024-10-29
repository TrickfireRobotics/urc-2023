# /bin/bash

# A script file `./container_launch.sh` that handles connecting to the `trickfirerobot` Docker container.

# You can add the following flags to the shell script:
# `-b` -> Force build the `trickfireimage`, even if it exists
# `-n` -> Force the `trickfireimage` image to be built without cache
# `-c` -> Force create the `trickfirerobot` container, even if it exists

text_helper="TRICKFIRE DOCKER LAUNCH"
trickire_container="trickfirerobot"
trickfire_image="trickfireimage"

# Colors
bold=$(tput bold)
BLUE='\033[0;34m'
NC='\033[0m' # No Color

b_flag='' # Force build docker container "-b"
no_cache_flag='' # Build docker container without cache "-n"
c_flag='' # Force create container "-c"

# Read the flags, if any were passed
while getopts 'bnc' flag; do
  case "${flag}" in
    b) b_flag='true' ;;
    n) no_cache_flag='true' ;;
    c) c_flag='true' ;;
    *) break ;;
  esac
done

# --- Shutdown the current trickfirerobot container if it is running
if [ "$( docker container inspect -f '{{.State.Running}}' ${trickire_container} )" = "true" ]; then
  echo -e "${BLUE}$(tput bold)[${text_helper}] Shutting down current \"${trickire_container}\" container${NC}"
  docker container kill ${trickire_container}
fi

# --- Handle the Docker image ---

# If the image does NOT exist OR we force the image to be built
if [ -z "$(docker images -q ${trickfire_image}:latest 2> /dev/null)" ] || [ "$b_flag" = true ]; then
  # Should we build without cache?
  if [ "$no_cache_flag" = true ]; then
    echo -e "${BLUE}$(tput bold)[${text_helper}] Building \"${trickfire_image}\" without cache${NC}"
    docker build --no-cache -t ${trickfire_image} -f .devcontainer/Dockerfile . 
  else
    echo -e "${BLUE}$(tput bold)[${text_helper}] Building \"${trickfire_image}\" with cache${NC}"
    docker build -t ${trickfire_image} -f .devcontainer/Dockerfile . 
  fi
else
    echo -e "${BLUE}$(tput bold)[${text_helper}] Image \"${trickfire_image}\" exists. Skipping${NC}"
fi


# Handle the Docker image

# If the container does NOT exist OR we force the container to be built
if [ ! "$(docker ps -a -q -f name=${trickire_container})" ]; then
    echo -e "${BLUE}$(tput bold)[${text_helper}] Container \"${trickire_container}\" does not exist. Creating${NC}"
    docker create --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" --name ${trickire_container} -it --privileged --network=host --workdir /home/trickfire/urc-2023 ${trickire_container}
else
    echo -e "${BLUE}$(tput bold)[${text_helper}] Container \"${trickire_container}\" does exist${NC}"
    # Should we remake the container?
    if [ "$c_flag" = true ]; then
        echo -e "${BLUE}$(tput bold)[${text_helper}] Container trickfire exists. Forcing removal and creation of a new \"${trickire_container}\" container${NC}"
        docker container rm ${trickire_container}
        docker create --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" --name ${trickire_container} -it --privileged --network=host --workdir /home/trickfire/urc-2023 ${trickire_container}

    fi
fi

#Start the container
echo -e "${BLUE}$(tput bold)[${text_helper}] Starting \"${trickire_container}\" container${NC}"
docker container restart ${trickire_container}

# Have the container take over the shell that executed this script
echo -e "${BLUE}$(tput bold)[${text_helper}] Connecting to \"${trickire_container}\" container${NC}"
docker exec -it ${trickire_container} /bin/bash
echo ""