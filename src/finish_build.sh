#!/bin/bash

# setup gui user with same uid and gid as host for X server access
# environment variables GUI_UID and GUI_GID will be set by host in run.sh
usermod --uid $GUI_UID gui
groupmod --gid $GUI_GID gui
echo "" >> /home/gui/.bashrc
echo "QT_X11_NO_MITSHM=1" >> /home/gui/.bashrc
echo "export QT_X11_NO_MITSHM" >> /home/gui/.bashrc

# Configure gui user permissions and environment to use ROS
echo "chowning opt (takes a minute)..."
chown -R gui:gui /opt
chown -R gui:gui /catkin_ws
chown -R gui:gui /src
chown -R gui:gui /root
echo "" >> /home/gui/.bashrc
echo "ROS_DISTRO=kinetic" >> /home/gui/.bashrc
echo "export ROS_DISTRO" >> /home/gui/.bashrc
echo "source /ros_entrypoint.sh" >> /home/gui/.bashrc

echo "done."


