#Build the docker image with the name "trickfireimage", and define the context of this build to be  
#the urc-2023 directory while refering the Dockerfile in .devcontainer
docker build -t trickfireimage -f .devcontainer/Dockerfile . 