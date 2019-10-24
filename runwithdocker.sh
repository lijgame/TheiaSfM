#!/bin/bash
echo "Image folder: $1"
cp applications/build_reconstruction_flags_jack.txt $1/flags.txt
docker run -it --rm --runtime=nvidia -u $(id -u):$(id -g) \
-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -v $XAUTHORITY:/.xauthority -e XAUTHORITY=/.xauthority \
-v $1:/data \
-v $PWD:/theiasfm \
-w /theiasfm \
lijgame/dev:theiasfm \
./reconstruct.sh /data
echo "$PWD"
