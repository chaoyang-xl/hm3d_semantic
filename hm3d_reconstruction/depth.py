from __future__ import annotations

import numpy as np


def depth_meters_to_uint16_mm(
    depth_m: np.ndarray, min_depth_m: float = 0.05, max_depth_m: float = 10.0
) -> np.ndarray:
    if min_depth_m < 0 or max_depth_m <= min_depth_m:
        raise ValueError("invalid depth range")
    depth = np.asarray(depth_m, dtype=np.float64)
    valid = (
        np.isfinite(depth) & (depth > 0) & (depth >= min_depth_m)
        & (depth <= max_depth_m)
    )
    result = np.zeros(depth.shape, dtype=np.uint16)
    result[valid] = np.clip(
        np.rint(depth[valid] * 1000.0), 1, np.iinfo(np.uint16).max
    ).astype(np.uint16)
    return result

