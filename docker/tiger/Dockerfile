FROM osrf/ros:kinetic-desktop-full

MAINTAINER Garrett Katz <garrett.katz@gmail.com>

# Add new sudo user for X11
# UID and GID will be assigned at runtime to match host
ENV USERNAME gui
RUN mkdir /etc/sudoers.d && \
    useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies for X11
RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        sudo \
        libxau-dev \
        libxdmcp-dev \
        libxcb1-dev \
        libxext-dev \
        libx11-dev \
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

# # Copy the nvidia hardware acceleration libraries from the "nvidia/opengl" container
# COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
#   /usr/local/lib/x86_64-linux-gnu \
#   /usr/local/lib/x86_64-linux-gnu

# COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
#   /usr/local/lib/i386-linux-gnu \
#   /usr/local/lib/i386-linux-gnu

# COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
#   /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json \
#   /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

# RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
#     echo '/usr/local/lib/i386-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
#     ldconfig

# Copy the tiger robot files to catkin workspace
CMD rm -rf catkin_ws

CMD mkdir -p catkin_ws/src

CMD source ./ros_entrypoint.sh

COPY robots/tiger/tiger_files/ /catkin_ws/src/tiger_files

COPY robots/tiger/tiger_application/ /catkin_ws/src/tiger_application

# Install ros kinetic packages
RUN apt update && \
      apt install -y ros-kinetic-gazebo-ros-pkgs ros-kinetic-teleop-twist-keyboard ros-kinetic-gazebo-ros-control \
     ros-kinetic-ros-controllers ros-kinetic-universal-robots

# # nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# copy source files
COPY src/ /src

# Set permissions for gui user
RUN chown -R gui:gui /src
RUN chown -R gui:gui /opt
RUN chown -R gui:gui /catkin_ws

CMD ["/src/startup.sh"]
# CMD ["/bin/bash"]

