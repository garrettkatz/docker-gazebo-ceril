#!/bin/bash

# setup new user with same uid and gid as host for X server access
# environment variables GUI_UID and GUI_GID will be set by host in run.sh
usermod --uid $GUI_UID $USERNAME
sudo groupmod --gid $GUI_GID $USERNAME # why Dockerfile installed sudo

# Enter bash shell
/bin/bash

