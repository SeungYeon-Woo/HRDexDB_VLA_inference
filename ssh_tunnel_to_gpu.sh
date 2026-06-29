#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: $0 <USER@INFERENCE_PC_HOST>" >&2
  echo "example: $0 seungyeon@192.168.0.104" >&2
  exit 2
fi

: "${LOCAL_PORT:=22610}"
: "${REMOTE_HOST:=127.0.0.1}"
: "${REMOTE_PORT:=22610}"
: "${SSH_PORT:=22}"

ssh -p "${SSH_PORT}" -N -L "${LOCAL_PORT}:${REMOTE_HOST}:${REMOTE_PORT}" "$1"
