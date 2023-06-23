#!/bin/bash

cd "$(dirname "$0")"
cd ..
export PRODUCTION_PATH=$PWD
export ARCH=`uname -m`
# export NUM_THREADS=`nproc`

xhost +
docker compose --env-file /home/oz/Downloads/theimagingsource_ros/docker/up.env \
    -f /home/oz/Downloads/theimagingsource_ros/docker/up.yml \
    up $@
xhost -
