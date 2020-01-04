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

GUI_UID=$(id -u)
GUI_GID=$(id -g)

docker run -it --rm \
    --volume="$XSOCK:$XSOCK:rw" \
    --volume="$XAUTH:$XAUTH:rw" \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --user="gui" \
    --env="GUI_UID=${GUI_UID}" \
    --env="GUI_GID=${GUI_GID}" \
    --name=tiger_ceril \
    docker-tiger-ceril
#     # --runtime=nvidia \
#     # docker-tiger-ceril
