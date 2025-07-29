#!/usr/bin/env bash

xhost +local:root

set -e

COMPOSE_FILE="docker-compose.yml"
SERVICE="bill_nav"

function do_run() {
  echo "🔹 Subindo serviço via docker-compose..."
  docker compose -f "$COMPOSE_FILE" up -d
  echo "🔹 Serviço '${SERVICE}' iniciado."
  docker compose -f "$COMPOSE_FILE" exec "$SERVICE" bash -l
}

function do_start() {
  echo "🔹 Iniciando container parado '$CONTAINER_NAME'..."
  docker compose -f "$COMPOSE_FILE" start
  docker compose -f "$COMPOSE_FILE" exec "$SERVICE" bash -l
}

function do_exec() {
  echo "🔹 Conectando ao container em execução '$CONTAINER_NAME'..."
  docker compose -f "$COMPOSE_FILE" exec "$SERVICE" bash -l
}

# Verifica status do container
STATUS=$(docker compose -f "$COMPOSE_FILE" ps --services --filter "status=running" | grep -w "$SERVICE" || true)

if [ -z "$STATUS" ]; then
  # não está rodando
  # verifica se existe (parado)
  EXISTS=$(docker compose -f "$COMPOSE_FILE" ps -a --services | grep -w "$SERVICE" || true)
  if [ -n "$EXISTS" ]; then
    do_start
  else
    do_run
  fi
else
  do_exec
fi
