#!/usr/bin/env bash
set -euo pipefail

: "${RLDX_ROOT:=/research/RLDX-1}"
: "${RLDX_PYTHON:=$RLDX_ROOT/.venv/bin/python}"
: "${CUDA_VISIBLE_DEVICES:=0}"
: "${RLDX_MODEL_PATH:=/research/ckpt/fk_lora_r16_b16_20k/fk_lora_r16_b16_20k/checkpoint-11000}"
: "${RLDX_BIND_HOST:=127.0.0.1}"
: "${RLDX_SERVER_PORT:=22610}"

cd "$RLDX_ROOT"

CUDA_VISIBLE_DEVICES="$CUDA_VISIBLE_DEVICES" \
NO_ALBUMENTATIONS_UPDATE=1 \
"$RLDX_PYTHON" rldx/eval/run_rldx_server.py \
  --model-path "$RLDX_MODEL_PATH" \
  --embodiment-tag GENERAL_EMBODIMENT \
  --host "$RLDX_BIND_HOST" \
  --port "$RLDX_SERVER_PORT" \
  --no-strict
