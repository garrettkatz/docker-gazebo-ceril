cd ../..

# Build image
docker build -f ./docker/poppy/Dockerfile -t="docker-poppy-ceril" .

# Remove previous container
docker rm -f poppy_ceril

# Configure container
GUI_UID=$(id -u)
GUI_GID=$(id -g)
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# # Might need this if xauth doesn't already exist
# if [ ! -f $XAUTH ]
# then
#     xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
#     if [ ! -z "$xauth_list" ]
#     then
#         echo $xauth_list | xauth -f $XAUTH nmerge -
#     else
#         touch $XAUTH
#     fi
#     chmod a+r $XAUTH
# fi
# # ---- or maybe ----
# touch $XAUTH
# xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -dit \
    --volume="$XSOCK:$XSOCK:rw" \
    --volume="$XAUTH:$XAUTH:rw" \
    --volume="/home/$USER/docker-gazebo-ceril/src:/src" \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="GUI_UID=${GUI_UID}" \
    --env="GUI_GID=${GUI_GID}" \
    --name=poppy_ceril \
    docker-poppy-ceril

docker exec poppy_ceril /src/finish_build.sh

cd docker/poppy/
