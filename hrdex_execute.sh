#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

: "${RLDX_SERVER_HOST:=127.0.0.1}"
: "${RLDX_SERVER_PORT:=22610}"
: "${INSTRUCTION:=grasp the apple and release it}"
: "${PARADEX_CAMERA:=}"

"${PYTHON:-python}" hrdex_execute_bridge.py \
  --server-host "$RLDX_SERVER_HOST" \
  --server-port "$RLDX_SERVER_PORT" \
  --instruction "$INSTRUCTION" \
  --paradex-root ../paradex \
  --paradex-camera-name "$PARADEX_CAMERA" \
  --dry-run \
  --execution-horizon 1 \
  "$@"
