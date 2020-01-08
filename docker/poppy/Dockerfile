FROM osrf/ros:kinetic-desktop-full

MAINTAINER Garrett Katz <garrett.katz@gmail.com>

RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libxau-dev \
        libxdmcp-dev \
        libxcb1-dev \
        libxext-dev \
        libx11-dev && \
    rm -rf /var/lib/apt/lists/*

# Copy the nvidia hardware acceleration libraries from the "nvidia/opengl" container
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
  /usr/local/lib/x86_64-linux-gnu \
  /usr/local/lib/x86_64-linux-gnu

COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
  /usr/local/lib/i386-linux-gnu \
  /usr/local/lib/i386-linux-gnu

COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    echo '/usr/local/lib/i386-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig

# Copy the poppy robot files to catkin workspace
CMD rm -rf poppy_ws

CMD mkdir -p poppy_ws/src

CMD source ./ros_entrypoint.sh

COPY robots/poppy/poppy_files/ /poppy_ws/src/poppy_files

COPY robots/poppy/poppy_application/ /poppy_ws/src/poppy_application

# Install ros kinetic packages
RUN apt update && \
      apt install -y ros-kinetic-gazebo-ros-pkgs ros-kinetic-teleop-twist-keyboard ros-kinetic-gazebo-ros-control \
     ros-kinetic-ros-controllers

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

CMD ["/bin/bash"]
