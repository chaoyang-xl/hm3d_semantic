from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional


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
    min_depth_m: float = 0.05
    max_depth_m: float = 10.0
    trajectory_mode: str = "waypoint"
    forward_step: float = 0.10
    turn_angle_deg: float = 5.0
    alignment_tolerance_deg: float = 10.0
    seed: int = 42
    save_semantic: bool = True
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
        if not 0 < self.hfov_deg < 180:
            raise ValueError("hfov must be in (0, 180)")
        if self.forward_step <= 0 or not 0 < self.turn_angle_deg <= 180:
            raise ValueError("invalid movement increments")
        if not 0 <= self.alignment_tolerance_deg <= 45:
            raise ValueError("alignment tolerance must be in [0, 45]")
        if self.min_depth_m < 0 or self.max_depth_m <= self.min_depth_m:
            raise ValueError("invalid depth range")
        if self.trajectory_mode not in {"waypoint", "replay"}:
            raise ValueError("first-stage exporter supports waypoint and replay")
        if self.trajectory_mode == "replay" and self.trajectory_file is None:
            raise ValueError("replay requires trajectory_file")

    def serializable(self) -> dict:
        data = asdict(self)
        for key, value in data.items():
            if isinstance(value, Path):
                data[key] = str(value)
        return data

