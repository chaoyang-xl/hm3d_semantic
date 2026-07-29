from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Union


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    scale: float = 1000.0

    def as_camera_dict(self) -> Dict[str, Union[int, float]]:
        return {
            "w": self.width, "h": self.height, "fx": self.fx, "fy": self.fy,
            "cx": self.cx, "cy": self.cy, "scale": self.scale,
        }


def compute_pinhole_intrinsics(
    width: int, height: int, hfov_deg: float
) -> CameraIntrinsics:
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if not 0.0 < hfov_deg < 180.0:
        raise ValueError("hfov_deg must be in (0, 180)")
    focal = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    return CameraIntrinsics(
        width, height, focal, focal, (width - 1) / 2.0, (height - 1) / 2.0
    )

