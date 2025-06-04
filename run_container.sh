xhost +local:docker
docker run -it \
  --net host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd)/ws":/home/ws \
  bill_mobile \
  /bin/bash

