#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hm3d_reconstruction.config import CaptureConfig
from hm3d_reconstruction.exporter import export_dataset


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Export synchronized HM3D RGB-D, semantic IDs, and GT poses",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--scene", required=True, type=Path,
        help="HM3D render asset, normally a .basis.glb file",
    )
    p.add_argument(
        "--scene-dataset-config", required=True, type=Path,
        help="Habitat scene_dataset_config.json used to load the scene",
    )
    p.add_argument(
        "--output", required=True, type=Path,
        help=(
            "output dataset directory; relative paths resolve against the current "
            "working directory; no default"
        ),
    )
    p.add_argument(
        "--frames", type=int, default=100,
        help=(
            "exact frame count for waypoint/replay; maximum recorded frame count "
            "for interactive mode"
        ),
    )
    p.add_argument("--width", type=int, default=640, help="image width in pixels")
    p.add_argument("--height", type=int, default=480, help="image height in pixels")
    p.add_argument(
        "--hfov-deg", type=float, default=79,
        help="horizontal camera field of view in degrees",
    )
    p.add_argument(
        "--sensor-height", type=float, default=0.88,
        help="camera height above the agent base in metres",
    )
    p.add_argument(
        "--display-scale", type=float, default=5.0,
        help="interactive preview scale only; saved image resolution is unchanged",
    )
    p.add_argument(
        "--ui-scale", type=float, default=0.0,
        help="controls panel/font scale; 0 detects it from screen height",
    )
    p.add_argument(
        "--min-depth-m", type=float, default=0.05,
        help="minimum valid saved depth in metres",
    )
    p.add_argument(
        "--max-depth-m", type=float, default=10,
        help="maximum valid saved depth in metres",
    )
    p.add_argument(
        "--trajectory-mode",
        choices=("waypoint", "interactive", "replay"),
        default="waypoint",
        help="camera trajectory source",
    )
    p.add_argument(
        "--trajectory-file", type=Path,
        help="JSON position/quaternion trajectory; required only for replay mode",
    )
    p.add_argument(
        "--forward-step", type=float, default=0.10,
        help="waypoint or keyboard movement distance per step in metres",
    )
    p.add_argument(
        "--turn-angle-deg", type=float, default=5,
        help="waypoint or keyboard rotation per step in degrees",
    )
    p.add_argument(
        "--alignment-tolerance-deg", type=float, default=10,
        help="waypoint heading tolerance before translation, in degrees",
    )
    p.add_argument("--seed", type=int, default=42, help="random start/path seed")
    p.add_argument(
        "--save-semantic",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="save Habitat semantic instance images; disable with --no-save-semantic",
    )
    p.add_argument(
        "--preview", action="store_true",
        help="write RGB/depth/semantic mosaics and a top-down trajectory image",
    )
    p.add_argument(
        "--overwrite", action="store_true",
        help="replace an existing non-empty output directory transactionally",
    )
    return p


def main() -> int:
    config = CaptureConfig(**vars(parser().parse_args()))
    output = export_dataset(config)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

