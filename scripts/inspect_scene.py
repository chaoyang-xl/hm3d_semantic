#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hm3d_reconstruction.config import CaptureConfig
from hm3d_reconstruction.simulator import HabitatCapture


def main() -> int:
    p = argparse.ArgumentParser(description="Inspect an HM3D-Semantic scene")
    p.add_argument("--scene", required=True, type=Path)
    p.add_argument("--scene-dataset-config", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    args = p.parse_args()
    config = CaptureConfig(args.scene, args.scene_dataset_config, args.output, frames=1)
    config.validate()
    source = HabitatCapture(config)
    try:
        frame = next(source.frames())
        report = {
            "scene": str(args.scene.resolve()),
            "navmesh_loaded": True,
            "navigable_area_m2": source.navigable_area,
            "semantic_instance_count": len(source.semantic_metadata["instances"]),
            "rgb_shape": list(frame.rgb.shape),
            "depth_shape": list(frame.depth_m.shape),
            "semantic_shape": list(frame.semantic.shape) if frame.semantic is not None else None,
        }
    finally:
        source.close()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"scene_report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

