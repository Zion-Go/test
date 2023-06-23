#!/bin/bash

cd "$(dirname "$0")"
cd ..
export PRODUCTION_PATH=$PWD
export ARCH=`uname -m`
# export NUM_THREADS=`nproc`

docker compose --env-file /home/oz/Downloads/theimagingsource_ros/docker/build.env \
    -f /home/oz/Downloads/theimagingsource_ros/docker/build.yml \
    up --build $@
