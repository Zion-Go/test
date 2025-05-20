FROM osrf/ros:foxy-desktop

# After FROM, enter the parent image from wich you want to build.
# We choose foxy-desktop.

# Currently, we are operating as root.

# Environment variable -> set language to C (computer) UTF-8 (8 bit unicode transformation format).
ENV LANG C.UTF-8

# Debconf is used to perform system-wide configutarions.
# Noninteractive -> use default settings -> put in debconf db.
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Set the nvidia container runtime.
#ENV NVIDIA_VISIBLE_DEVICES \
 #   ${NVIDIA_VISIBLE_DEVICES:-all}
#ENV NVIDIA_DRIVER_CAPABILITIES \
 #   ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Environment variable -> see output in real time.
ENV PYTHONUNBUFFERED 1

# Install some handy tools.
RUN set -x \
        && apt-get update \
        && apt-get upgrade -y \
        && apt-get install -y apt-utils \
        && apt-get install -y mesa-utils \
        && apt-get install -y iputils-ping \
        && apt-get install -y apt-transport-https ca-certificates \
        && apt-get install -y openssh-server python3-pip exuberant-ctags \
        && apt-get install -y git vim tmux nano htop sudo curl wget gnupg2 \
        && apt-get install -y bash-completion \
        && apt-get install -y python3-psycopg2 \
        && rm -rf /var/lib/apt/lists/* \
        && useradd -ms /bin/bash user \
        && echo "user:user" | chpasswd && adduser user sudo \
        && echo "user ALL=(ALL) NOPASSWD: ALL " >> /etc/sudoers

# The OSRF container didn't link python3 to python, causing ROS scripts to fail.
RUN ln -s /usr/bin/python3 /usr/bin/python

# Set USER to user + define working directory.
USER user
WORKDIR /home/user

# install ROS pkg



# setup ROS
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/user/.bashrc

# gazebo
# RUN curl -sSL http://get.gazebosim.org | sh
RUN sudo apt-get update && sudo apt-get install -y ros-foxy-gazebo-ros-pkgs

