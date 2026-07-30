from __future__ import annotations

from dataclasses import replace

import numpy as np

from .occupancy import RosOccupancyMap


UNKNOWN = np.uint8(205)
FREE = np.uint8(254)
OCCUPIED = np.uint8(0)


def simulate_lidar_exploration(
    ground_truth: RosOccupancyMap,
    positions_xy: np.ndarray,
    ray_count: int = 720,
    max_range_m: float = 10.0,
) -> tuple[RosOccupancyMap, dict]:
    """Reveal a ground-truth grid with ideal 360-degree planar lidar rays."""

    positions = np.asarray(positions_xy, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 2:
        raise ValueError("map trajectory positions must have shape Nx2")
    if not len(positions) or not np.isfinite(positions).all():
        raise ValueError("map trajectory positions must be finite and non-empty")
    if ray_count < 4:
        raise ValueError("map ray count must be at least 4")
    if max_range_m <= 0.0:
        raise ValueError("map maximum range must be positive")

    image = np.full(ground_truth.image.shape, UNKNOWN, dtype=np.uint8)
    pixel_positions = []
    for x, y in positions:
        pixel = ground_truth.world_to_pixel(float(x), float(y))
        if pixel is not None:
            pixel_positions.append(pixel)
    unique_positions = list(dict.fromkeys(pixel_positions))
    if not unique_positions:
        raise RuntimeError("recorded trajectory does not intersect the ROS map")

    angles = np.linspace(0.0, 2.0 * np.pi, ray_count, endpoint=False)
    delta_columns = np.cos(angles)
    delta_rows = -np.sin(angles)
    maximum_steps = int(np.ceil(max_range_m / ground_truth.resolution))
    height, width = image.shape

    for start_row, start_column in unique_positions:
        if ground_truth.image[start_row, start_column] != FREE:
            continue
        image[start_row, start_column] = FREE
        active = np.ones(ray_count, dtype=bool)
        for step in range(1, maximum_steps + 1):
            rows = np.rint(start_row + delta_rows * step).astype(np.int32)
            columns = np.rint(
                start_column + delta_columns * step
            ).astype(np.int32)
            valid = (
                active
                & (rows >= 0)
                & (rows < height)
                & (columns >= 0)
                & (columns < width)
            )
            active &= valid
            ray_indices = np.flatnonzero(active)
            if not len(ray_indices):
                break
            ray_rows = rows[ray_indices]
            ray_columns = columns[ray_indices]
            is_free = (
                ground_truth.image[ray_rows, ray_columns] == FREE
            )
            if np.any(is_free):
                image[
                    ray_rows[is_free], ray_columns[is_free]
                ] = FREE
            hits = ~is_free
            if np.any(hits):
                image[ray_rows[hits], ray_columns[hits]] = OCCUPIED
                active[ray_indices[hits]] = False

    explored = replace(ground_truth, image=image)
    counts = {
        "map_mode": "explored",
        "ray_count": int(ray_count),
        "max_range_m": float(max_range_m),
        "unique_scan_positions": len(unique_positions),
        "free_cells": int(np.count_nonzero(image == FREE)),
        "occupied_cells": int(np.count_nonzero(image == OCCUPIED)),
        "unknown_cells": int(np.count_nonzero(image == UNKNOWN)),
    }
    counts["explored_cells"] = counts["free_cells"] + counts["occupied_cells"]
    counts["explored_ratio"] = (
        counts["explored_cells"] / image.size
    )
    return explored, counts
