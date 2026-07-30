from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class CaptureConfig:
    scene: Path
    scene_dataset_config: Path
    output: Path
    frames: int = 100
    width: int = 640
    height: int = 480
    hfov_deg: float = 79.0
    sensor_height: float = 0.88
    display_scale: float = 5.0
    ui_scale: float = 0.0
    min_depth_m: float = 0.05
    max_depth_m: float = 10.0
    trajectory_mode: str = "waypoint"
    forward_step: float = 0.10
    turn_angle_deg: float = 5.0
    alignment_tolerance_deg: float = 10.0
    seed: int = 42
    save_semantic: bool = True
    export_ros_map: bool = False
    map_resolution_m: float = 0.05
    map_floor_height_m: Optional[float] = None
    map_height_tolerance_m: float = 0.20
    map_min_island_area_m2: float = 1.0
    map_mode: str = "ground_truth"
    map_ray_count: int = 720
    map_max_range_m: float = 10.0
    preview: bool = False
    overwrite: bool = False
    trajectory_file: Optional[Path] = None

    def validate(self) -> None:
        if not self.scene.is_file():
            raise ValueError(f"scene not found: {self.scene}")
        if not self.scene_dataset_config.is_file():
            raise ValueError(f"scene dataset config not found: {self.scene_dataset_config}")
        if self.frames <= 0 or self.width <= 0 or self.height <= 0:
            raise ValueError("frames and resolution must be positive")
        if not 0.5 <= self.display_scale <= 6.0:
            raise ValueError("display_scale must be in [0.5, 6.0]")
        if self.ui_scale != 0.0 and not 0.8 <= self.ui_scale <= 3.0:
            raise ValueError("ui_scale must be 0 (auto) or in [0.8, 3.0]")
        if not 0 < self.hfov_deg < 180:
            raise ValueError("hfov must be in (0, 180)")
        if self.forward_step <= 0 or not 0 < self.turn_angle_deg <= 180:
            raise ValueError("invalid movement increments")
        if not 0 <= self.alignment_tolerance_deg <= 45:
            raise ValueError("alignment tolerance must be in [0, 45]")
        if self.min_depth_m < 0 or self.max_depth_m <= self.min_depth_m:
            raise ValueError("invalid depth range")
        if self.map_resolution_m <= 0:
            raise ValueError("map_resolution_m must be positive")
        if self.map_height_tolerance_m <= 0:
            raise ValueError("map_height_tolerance_m must be positive")
        if (
            self.map_floor_height_m is not None
            and not np.isfinite(self.map_floor_height_m)
        ):
            raise ValueError("map_floor_height_m must be finite")
        if self.map_min_island_area_m2 < 0:
            raise ValueError("map_min_island_area_m2 must be non-negative")
        if self.map_mode not in {"ground_truth", "explored"}:
            raise ValueError("map_mode must be ground_truth or explored")
        if self.map_ray_count < 4:
            raise ValueError("map_ray_count must be at least 4")
        if self.map_max_range_m <= 0:
            raise ValueError("map_max_range_m must be positive")
        if self.trajectory_mode not in {"waypoint", "interactive", "replay"}:
            raise ValueError(
                "trajectory_mode must be waypoint, interactive, or replay"
            )
        if self.trajectory_mode == "replay" and self.trajectory_file is None:
            raise ValueError("replay requires trajectory_file")

    def serializable(self) -> dict:
        data = asdict(self)
        for key, value in data.items():
            if isinstance(value, Path):
                data[key] = str(value)
        return data

