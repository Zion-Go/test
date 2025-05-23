#!/bin/bash

#if [ ! -d "Commands" ]
#then
 #   mkdir -p "Commands/bin"
#fi

#if [ ! -d "Projects/dev_ws_src" ]
#then
 #   mkdir -p "Projects/dev_ws_src"
#fi

#if [ ! -d "ExampleCode" ]
#then
 #   mkdir -p "ExampleCode"
#fi

#if ! command -v glxinfo &> /dev/null
#then
 #   echo "glxinfo command  not found! Execute \'sudo apt install mesa-utils\' to install it."
  #  exit
#fi

# vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`

xhost +local:docker

# --device=/dev/video0:/dev/video0
# For non root usage:
# RUN sudo usermod -a -G video developer

if [ $vendor = "NVIDIA" ]; then
    docker run -it --rm \
        --name ros2_foxy_desktop \
        --hostname ros2_foxy_desktop \
        --device /dev/snd \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --gpus all \
        ros2_foxy_docker:latest \
        bash
else
    docker run --privileged -it --rm \
        --name ros2_foxy_desktop \
        --hostname ros2_foxy_desktop \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix \
        --device=/dev/dri:/dev/dri \
        --env="DISPLAY=$DISPLAY" \
        -e "TERM=xterm-256color" \
        --cap-add SYS_ADMIN --device /dev/fuse \
        ros2_foxy_docker:latest \
        bash
fi
