#!/bin/bash

run_docker() {
    xhost +local:root # Giving display privileges

    # -it is for interactive, tty
    # --privileged for accessing /dev contents
    # --net=host to share the same network as host machine. TL;DR same IP.
    docker run -it --privileged --net=host \
    --name limo_bot \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v $(dirname "$0")/app.sh:/root/app.sh \
    $@
}

stop_docker() {
    docker stop limo_bot && docker rm limo_bot
    xhost -local:root # Remove display privileges
}