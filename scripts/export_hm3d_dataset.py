#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hm3d_reconstruction.config import CaptureConfig
from hm3d_reconstruction.exporter import export_dataset


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Export synchronized HM3D GT RGB-D-Semantic")
    p.add_argument("--scene", required=True, type=Path)
    p.add_argument("--scene-dataset-config", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--frames", type=int, default=100)
    p.add_argument("--width", type=int, default=640); p.add_argument("--height", type=int, default=480)
    p.add_argument("--hfov-deg", type=float, default=79); p.add_argument("--sensor-height", type=float, default=0.88)
    p.add_argument("--min-depth-m", type=float, default=0.05); p.add_argument("--max-depth-m", type=float, default=10)
    p.add_argument("--trajectory-mode", choices=("waypoint","replay"), default="waypoint")
    p.add_argument("--trajectory-file", type=Path)
    p.add_argument("--forward-step", type=float, default=0.10); p.add_argument("--turn-angle-deg", type=float, default=5)
    p.add_argument("--alignment-tolerance-deg", type=float, default=10)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--save-semantic", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--preview", action="store_true"); p.add_argument("--overwrite", action="store_true")
    return p


def main() -> int:
    config = CaptureConfig(**vars(parser().parse_args()))
    output = export_dataset(config)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

