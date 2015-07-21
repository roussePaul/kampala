## Script to create a new docker for the simulated drone
# DON'T RUN IT LIKE THIS
# copy these command and adapt them to create the drone you want

docker run --privileged -e DISPLAY=$DISPLAY --name=docker-iris-1 -it px4io/px4-ros-full bash 
docker start docker-iris-1
docker exec -it docker-iris-1 bash -c "`cat init_ws_docker.sh`"
