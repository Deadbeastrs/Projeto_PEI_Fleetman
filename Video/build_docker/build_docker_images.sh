#!/bin/bash
echo Building x264 image...
docker build -t video --rm --build-arg TMP_BUILD_ARG=video -f video.dockerfile ..
docker system prune -f
