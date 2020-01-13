#!/bin/bash

# # setup gui user with same uid and gid as host for X server access
# # environment variables GUI_UID and GUI_GID will be set by host in run.sh
# usermod --uid $GUI_UID gui
# groupmod --gid $GUI_GID gui

# # Enter bash shell as gui user
sudo -u gui /src/gui_startup.sh
# bash
