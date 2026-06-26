#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

: "${RLDX_SERVER_HOST:=127.0.0.1}"
: "${RLDX_SERVER_PORT:=22610}"
: "${INSTRUCTION:=grasp the apple and release it}"
: "${PARADEX_CAMERA:=22645029}"
: "${LOG_JSONL:=hrdex_dryrun_log.jsonl}"

"${PYTHON:-python}" hrdex_rldx_dryrun_bridge.py \
  --server-host "$RLDX_SERVER_HOST" \
  --server-port "$RLDX_SERVER_PORT" \
  --instruction "$INSTRUCTION" \
  --paradex-root ../paradex \
  --paradex-camera-name "$PARADEX_CAMERA" \
  --arm-state-topic /right/xarm/joint_states \
  --xarm-robot-states-topic /right/xarm/robot_states \
  --hand-backend paradex_direct \
  --log-jsonl "$LOG_JSONL" \
  "$@"
