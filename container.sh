#!/usr/bin/env bash

xhost +local:root

set -e

CONTAINER_NAME="bill"
IMAGE_NAME="bill" 

function do_run() {
  echo "🔹 Criando e iniciando container '$CONTAINER_NAME'..."
  docker run -it --name "$CONTAINER_NAME" \
    --volume="$(pwd)/nav_ws:/nav_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network=host \
    --privileged \
    "$IMAGE_NAME" bash -l
}

function do_start() {
  echo "🔹 Iniciando container parado '$CONTAINER_NAME'..."
  docker start "$CONTAINER_NAME"
  docker exec -it "$CONTAINER_NAME" bash -l
}

function do_exec() {
  echo "🔹 Conectando ao container em execução '$CONTAINER_NAME'..."
  docker exec -it "$CONTAINER_NAME" bash -l
}

# Verifica status do container
STATUS=$(docker ps --filter "name=^/${CONTAINER_NAME}$" --filter "status=running" -q)

if [ -z "$STATUS" ]; then
  # não está rodando
  # verifica se existe (parado)
  EXISTS=$(docker ps -a --filter "name=^/${CONTAINER_NAME}$" -q)
  if [ -n "$EXISTS" ]; then
    do_start
  else
    do_run
  fi
else
  do_exec
fi
