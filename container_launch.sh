# /bin/bash

text_helper="TRICKFIRE DOCKER LAUNCH"

bold=$(tput bold)
BLUE='\033[0;34m'
NC='\033[0m' # No Color

b_flag='' # Force build docker container "-b"
no_cache_flag='' # Build docker container without cache "-n"
c_flag='' # Force create container "-c"

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
    docker create --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" --name trickfirerobot -i --privileged --network=host --workdir /home/trickfire/urc-2023 trickfireimage
else
    echo -e "${BLUE}$(tput bold)[${text_helper}] Container \"trickfirerobot\" does exist${NC}"
    if [ "$c_flag" = true ]; then
        echo -e "${BLUE}$(tput bold)[${text_helper}] Container trickfire exists. Forcing removal and creation of a new trickfirerobot container${NC}"
        docker container rm trickfirerobot
        docker create --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" --name trickfirerobot -t --privileged --network=host --workdir /home/trickfire/urc-2023 trickfireimage

    fi
fi

#start the container
echo -e "${BLUE}$(tput bold)[${text_helper}] Starting trickfirerobot container${NC}"
docker container restart trickfirerobot

echo -e "${BLUE}$(tput bold)[${text_helper}] Connecting to trickfirerobot container${NC}"
docker exec -it trickfirerobot /bin/bash
echo ""