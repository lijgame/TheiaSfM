#!/bin/bash

docker run -it --rm --runtime=nvidia -u $(id -u):$(id -g) \
-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -v $XAUTHORITY:/.xauthority -e XAUTHORITY=/.xauthority \
-v $PWD:/theiasfm \
-w /theiasfm \
lijgame/dev:theiasfm \
./build.sh