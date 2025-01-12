# /bin/bash

# A script file `./container_launch.sh` that handles connecting to the `trickfirerobot` Docker container.

# You can add the following flags to the shell script:
# `-b` -> Force build the `trickfireimage`, even if it exists
# `-n` -> Force the `trickfireimage` image to be built without cache
# `-c` -> Force create the `trickfirerobot` container, even if it exists
# '-m' -> Force the script to use the master image and container

text_helper="TRICKFIRE DOCKER LAUNCH"
trickire_container="trickfirerobot"
trickfire_image="trickfireimage"

# Colors
bold=$(tput bold)
BLUE='\033[0;34m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

b_flag='' # Force build docker container "-b"
no_cache_flag='' # Build docker container without cache "-n"
c_flag='' # Force create container "-c"
master_flag='' #Use the master image and container

# Read the flags, if any were passed
while getopts 'bncm' flag; do
  case "${flag}" in
    b) b_flag='true' ;;
    n) no_cache_flag='true' ;;
    c) c_flag='true' ;;
    m) master_flag='true' ;;
    *) break ;;
  esac
done

if [ "$master_flag" = true ]; then
  echo -e "${BLUE}$(tput bold)[${text_helper}] Using master image and container${NC}"
  trickire_container="master_trickfirerobot"
  trickfire_image="master_trickfireimage"
fi



# --- Shutdown both "master_trickfirerobot" and/or "trickfirerobot" if they are running

# --- Shutdown the current "master_trickfirerobot" container if it is running
if [ "$( docker container inspect -f '{{.State.Running}}' master_trickfirerobot )" = "true" ]; then
  echo -e "${BLUE}$(tput bold)[${text_helper}] Shutting down current \"master_trickfirerobot\" container${NC}"
  
  # --- Feed stdin and stderr into the same variable
  result=$(docker container kill master_trickfirerobot 2>&1)

  if [ "$result" = "master_trickfirerobot" ]; then
    echo -e "${GREEN}$(tput bold)[${text_helper}] SHUTDOWN OF 'master_trickfirerobot' SUCCESS${NC}"
  else
    echo -e "${RED}$(tput bold)[${text_helper}] ERROR: ${result}${NC}"
    echo -e "${RED}$(tput bold)[${text_helper}] EXITING${NC}"
    exit 1
  fi
fi

# --- Shutdown the current "trickfirerobot" container if it is running
if [ "$( docker container inspect -f '{{.State.Running}}' trickfirerobot )" = "true" ]; then
  echo -e "${BLUE}$(tput bold)[${text_helper}] Shutting down current \"trickfirerobot\" container${NC}"

  # --- Feed stdin and stderr into the same variable
  result=$(docker container kill trickfirerobot 2>&1)

  if [ "$result" = "trickfirerobot" ]; then
    echo -e "${GREEN}$(tput bold)[${text_helper}] SHUTDOWN of 'trickfirerobot' SUCCESS${NC}"
  else
    echo -e "${RED}$(tput bold)[${text_helper}] ERROR: ${result}${NC}"
    echo -e "${RED}$(tput bold)[${text_helper}] EXITING${NC}"
    exit 1
  fi
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
    echo -e "${BLUE}$(tput bold)[${text_helper}] Image \"${trickfire_image}\" exists. Skipping building process${NC}"
fi


# Handle the Docker image

# If the container does NOT exist OR we force the container to be built
if [ ! "$(docker ps -a -q -f name=${trickire_container})" ]; then
    echo -e "${BLUE}$(tput bold)[${text_helper}] Container \"${trickire_container}\" does not exist. Creating${NC}"
    docker create --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" --name ${trickire_container} -it --privileged --network=host --workdir /home/trickfire/urc-2023 ${trickfire_image}
else
    echo -e "${BLUE}$(tput bold)[${text_helper}] Container \"${trickire_container}\" does exist${NC}"
    # Should we remake the container?
    if [ "$c_flag" = true ]; then
        echo -e "${BLUE}$(tput bold)[${text_helper}] Container trickfire exists. Forcing removal and creation of a new \"${trickire_container}\" container${NC}"
        docker container rm ${trickire_container}
        docker create --mount type=bind,source="$(pwd)",target="/home/trickfire/urc-2023" --name ${trickire_container} -it --privileged --network=host --workdir /home/trickfire/urc-2023 ${trickfire_image}

    fi
fi

#Start the container
echo -e "${BLUE}$(tput bold)[${text_helper}] Starting \"${trickire_container}\" container${NC}"
# --- Feed stdin and stderr into the same variable
restartResult=$(docker container restart ${trickire_container} 2>&1)

# Check to make sure the container launched correctly
if [ "$restartResult" = "$trickire_container" ]; then
  echo -e "${GREEN}$(tput bold)[${text_helper}] LAUNCH of '${trickire_container}' SUCCESS${NC}"
else
  echo -e "${RED}$(tput bold)[${text_helper}] ERROR: ${restartResult}${NC}"
  echo -e "${RED}$(tput bold)[${text_helper}] EXITING${NC}"
  exit 1
fi

# Have the container take over the shell that executed this script
echo -e "${BLUE}$(tput bold)[${text_helper}] Connecting to \"${trickire_container}\" container${NC}"
docker exec -it ${trickire_container} /bin/bash
echo ""