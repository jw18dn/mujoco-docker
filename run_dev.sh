#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

PROJECT_NAME="jason"

docker build -t ${PROJECT_NAME} "${SCRIPT_DIR}"

xhost +

docker run -it --rm \
    -v $SCRIPT_DIR:/app \
    -w /app \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ${PROJECT_NAME} \
    bash