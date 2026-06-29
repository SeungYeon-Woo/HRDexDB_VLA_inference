#!/usr/bin/env python3
"""Print raw paradex camera stream keys received on the robot PC."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def parse_pc_list(value: str) -> list[str] | None:
    if not value:
        return None
    return [x.strip() for x in value.split(",") if x.strip()]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--paradex-root", default="../paradex")
    parser.add_argument("--pc-list", default="", help="Comma-separated capture PC names. Empty uses paradex config.")
    parser.add_argument("--hz", type=float, default=2.0)
    parser.add_argument("--samples", type=int, default=0, help="0 means run forever.")
    args = parser.parse_args()

    root = Path(args.paradex_root).expanduser().resolve()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

    from paradex.io.capture_pc.data_sender import DataCollector

    collector = DataCollector(pc_list=parse_pc_list(args.pc_list))
    collector.start()
    period = 1.0 / max(args.hz, 1e-6)

    try:
        count = 0
        while args.samples <= 0 or count < args.samples:
            data = collector.get_data()
            image_keys = [k for k, v in data.items() if isinstance(v, dict) and v.get("type") == "image"]
            summary = {}
            for key in image_keys:
                item = data[key]
                payload = item.get("data")
                summary[str(key)] = {
                    "frame_id": item.get("frame_id"),
                    "timestamp": item.get("timestamp"),
                    "shape": item.get("shape"),
                    "encoding": item.get("encoding"),
                    "bytes": None if payload is None else len(payload),
                }
            print({
                "available": [str(k) for k in data.keys()],
                "image_keys": [str(k) for k in image_keys],
                "images": summary,
            }, flush=True)
            count += 1
            if args.samples <= 0 or count < args.samples:
                time.sleep(period)
    finally:
        collector.end()


if __name__ == "__main__":
    main()
