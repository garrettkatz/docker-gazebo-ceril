cd ../..

# Build image
docker build -f ./docker/poppy/Dockerfile -t="docker-poppy-ceril" .

# Remove previous container
docker rm -f poppy_ceril

# Configure container
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
GUI_UID=$(id -u)
GUI_GID=$(id -g)
docker run -dit \
    --volume="$XSOCK:$XSOCK:rw" \
    --volume="$XAUTH:$XAUTH:rw" \
    --volume="/home/outpost/docker-gazebo-ceril/src:/src" \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="GUI_UID=${GUI_UID}" \
    --env="GUI_GID=${GUI_GID}" \
    --name=poppy_ceril \
    docker-poppy-ceril

docker exec poppy_ceril /src/finish_build.sh

cd docker/poppy/
