#!/usr/bin/env bash
set -euo pipefail

PARADEX_ROOT="${PARADEX_ROOT:-../paradex}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$SCRIPT_DIR/camera/stream_client_hrdex.py"
DST_DIR="$PARADEX_ROOT/src/capture/camera"
DST="$DST_DIR/stream_client_hrdex.py"

if [ ! -f "$SRC" ]; then
  echo "missing $SRC" >&2
  exit 1
fi

mkdir -p "$DST_DIR"
cp "$SRC" "$DST"
chmod +x "$DST"

echo "installed $DST"
echo "camera PC run command:"
echo "  cd $PARADEX_ROOT"
echo "  python src/capture/camera/stream_client_hrdex.py --fps 10"
