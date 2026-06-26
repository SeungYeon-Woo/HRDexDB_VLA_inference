#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
: "${RLDX_ROOT:=$SCRIPT_DIR/third_party/RLDX-1}"
: "${RLDX_PYTHON:=$RLDX_ROOT/.venv/bin/python}"
: "${CUDA_VISIBLE_DEVICES:=0}"
: "${RLDX_MODEL_PATH:=$HOME/VCL/checkpoints/HRDexDB_RLDX/checkpoint-11000}"
: "${RLDX_BIND_HOST:=127.0.0.1}"
: "${RLDX_SERVER_PORT:=22610}"

if [ ! -d "$RLDX_MODEL_PATH" ]; then
  echo "RLDX_MODEL_PATH does not exist: $RLDX_MODEL_PATH" >&2
  echo "Download the checkpoint first:" >&2
  echo "  cd $SCRIPT_DIR && ./download_checkpoint.sh" >&2
  exit 1
fi

cd "$RLDX_ROOT"

CUDA_VISIBLE_DEVICES="$CUDA_VISIBLE_DEVICES" \
NO_ALBUMENTATIONS_UPDATE=1 \
"$RLDX_PYTHON" rldx/eval/run_rldx_server.py \
  --model-path "$RLDX_MODEL_PATH" \
  --embodiment-tag GENERAL_EMBODIMENT \
  --host "$RLDX_BIND_HOST" \
  --port "$RLDX_SERVER_PORT" \
  --no-strict
