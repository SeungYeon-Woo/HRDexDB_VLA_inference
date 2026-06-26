#!/usr/bin/env python3
"""Print Inspire F1 direct-controller state for VLA dry-run setup."""

from __future__ import annotations

import argparse
import time
import numpy as np

from xarm_inspire_rldx_bridge import ParadexDirectInspireHand


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--paradex-root", default="../paradex")
    parser.add_argument("--tactile", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--hz", type=float, default=2.0)
    parser.add_argument("--samples", type=int, default=0, help="Number of samples to print; 0 means run forever.")
    args = parser.parse_args()

    hand = ParadexDirectInspireHand(
        paradex_root=args.paradex_root,
        tactile=args.tactile,
    )
    period = 1.0 / args.hz
    try:
        count = 0
        while args.samples <= 0 or count < args.samples:
            qpos = hand.get_qpos()
            raw = hand.get_raw_angle()
            print({
                "qpos_rad": None if qpos is None else np.round(qpos, 4).tolist(),
                "raw_angle_est": None if raw is None else np.round(raw, 2).tolist(),
            }, flush=True)
            count += 1
            if args.samples <= 0 or count < args.samples:
                time.sleep(period)
    finally:
        hand.close()


if __name__ == "__main__":
    main()
