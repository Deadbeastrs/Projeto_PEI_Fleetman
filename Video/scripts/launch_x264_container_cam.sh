#!/bin/bash
docker run \
    --rm \
    --network host\
    --privileged \
    --name video \
    --device=/dev/video0:/dev/video0 \
    -v /dev/snd:/dev/snd \
    -it video \
