# /bin/bash

# A script file `./container_launch.sh` that handles connecting to the `trickfirerobot` Docker container.

# You can add the following flags to the shell script:
# `-b` -> Force build the `trickfireimage`, even if it exists
# `-n` -> Force the `trickfireimage` image to be built without cache
# `-c` -> Force create the `trickfirerobot` container, even if it exists

text_helper="TRICKFIRE DOCKER LAUNCH"

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

# --- Handle the Docker image ---

# If the image does NOT exist OR we force the image to be built
if [ -z "$(docker images -q trickfireimage:latest 2> /dev/null)" ] || [ "$b_flag" = true ]; then
  # Should we build without cache?
  if [ "$no_cache_flag" = true ]; then
    echo -e "${BLUE}$(tput bold)[${text_helper}] Building trickfireimage without cache${NC}"
    docker build --no-cache -t trickfireimage -f .devcontainer/Dockerfile . 
  else
    echo -e "${BLUE}$(tput bold)[${text_helper}] Building trickfireimage with cache${NC}"
    docker build -t trickfireimage -f .devcontainer/Dockerfile . 
  fi
else
    echo -e "${BLUE}$(tput bold)[${text_helper}] Image trickfireimage exists. Skipping${NC}"
fi


# Handle the Docker image

# If the container does NOT exist OR we force the container to be built
if [ ! "$(docker ps -a -q -f name=trickfirerobot)" ]; then
    echo -e "${BLUE}$(tput bold)[${text_helper}] Container trickfirerobot does not exist. Creating${NC}"
    docker create --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" --name trickfirerobot -it --privileged --network=host --workdir /home/trickfire/urc-2023 trickfireimage
else
    echo -e "${BLUE}$(tput bold)[${text_helper}] Container \"trickfirerobot\" does exist${NC}"
    # Should we remake the container?
    if [ "$c_flag" = true ]; then
        echo -e "${BLUE}$(tput bold)[${text_helper}] Container trickfire exists. Forcing removal and creation of a new trickfirerobot container${NC}"
        docker container rm trickfirerobot
        docker create --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" --name trickfirerobot -it --privileged --network=host --workdir /home/trickfire/urc-2023 trickfireimage

    fi
fi

#Start the container
echo -e "${BLUE}$(tput bold)[${text_helper}] Starting trickfirerobot container${NC}"
docker container restart trickfirerobot

# Have the container take over the shell that executed this script
echo -e "${BLUE}$(tput bold)[${text_helper}] Connecting to trickfirerobot container${NC}"
docker exec -it trickfirerobot /bin/bash
echo ""