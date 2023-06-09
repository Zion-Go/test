#!/bin/bash

cd "$(dirname "$0")"
cd ..
export PRODUCTION_PATH=$PWD
export ARCH=`uname -m`
# export NUM_THREADS=`nproc`

docker compose --env-file $PRODUCTION_PATH/ros2_humble_docker/build.env \
    -f $PRODUCTION_PATH/ros2_humble_docker/build.yml \
    up --build $@
