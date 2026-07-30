from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


@dataclass(frozen=True)
class RosOccupancyMap:
    """ROS map_server-compatible raster in the project's Z-up map frame."""

    image: np.ndarray
    resolution: float
    origin: tuple[float, float, float]
    floor_height_habitat: float
    height_tolerance: float
    reference_island_index: int
    included_island_indices: tuple[int, ...]

    @property
    def height(self) -> int:
        return int(self.image.shape[0])

    @property
    def width(self) -> int:
        return int(self.image.shape[1])

    def world_to_pixel(self, x: float, y: float) -> tuple[int, int] | None:
        column = int(np.floor((float(x) - self.origin[0]) / self.resolution))
        local_y = (float(y) - self.origin[1]) / self.resolution
        row = self.height - 1 - int(np.floor(local_y))
        if row < 0 or row >= self.height or column < 0 or column >= self.width:
            return None
        return row, column

    def metadata(self) -> dict:
        free_cells = int(np.count_nonzero(self.image == 254))
        return {
            "yaml": "map.yaml",
            "image": "map.pgm",
            "resolution": self.resolution,
            "origin": list(self.origin),
            "width": self.width,
            "height": self.height,
            "floor_height_habitat": self.floor_height_habitat,
            "height_tolerance": self.height_tolerance,
            "reference_island_index": self.reference_island_index,
            "included_island_indices": list(self.included_island_indices),
            "free_cells": free_cells,
            "free_area_m2": free_cells * self.resolution ** 2,
            "coordinate_conversion": "x_map=x_habitat,y_map=-z_habitat",
        }


def occupancy_from_pathfinder(
    pathfinder: Any,
    resolution: float,
    floor_height: float,
    height_tolerance: float,
    reference_position: np.ndarray,
    min_island_area: float = 1.0,
) -> RosOccupancyMap:
    """Rasterize usable NavMesh islands on the recorded floor."""

    if resolution <= 0.0:
        raise ValueError("map resolution must be positive")
    if height_tolerance <= 0.0:
        raise ValueError("map height tolerance must be positive")
    reference = np.asarray(reference_position, dtype=np.float64)
    if reference.shape != (3,) or not np.isfinite(reference).all():
        raise ValueError("map reference position must be a finite XYZ point")
    if min_island_area < 0.0:
        raise ValueError("minimum island area must be non-negative")

    lower, _ = pathfinder.get_bounds()
    lower = np.asarray(lower, dtype=np.float64)
    islands = np.asarray(
        pathfinder.get_topdown_island_view(
            float(resolution), float(floor_height), float(height_tolerance)
        ),
        dtype=np.int32,
    )
    if islands.ndim != 2 or not islands.size:
        raise RuntimeError("PathFinder returned an empty top-down island map")

    reference_island = int(pathfinder.get_island(reference))
    present_islands = [int(value) for value in np.unique(islands) if value >= 0]
    included_islands = tuple(
        value for value in present_islands
        if value == reference_island
        or float(pathfinder.island_area(value)) >= float(min_island_area)
    )
    free = np.isin(islands, included_islands)
    if not np.any(free):
        raise RuntimeError(
            f"no usable NavMesh island exists at floor height "
            f"{floor_height:.6f}"
        )

    image = np.where(free, 254, 0).astype(np.uint8)
    # Habitat rows increase with +Z. With y_map=-z_habitat, ROS's bottom-up
    # image convention maps those rows back to the same stored PGM rows.
    origin_y = -float(lower[2]) - image.shape[0] * float(resolution)
    return RosOccupancyMap(
        image=image,
        resolution=float(resolution),
        origin=(float(lower[0]), origin_y, 0.0),
        floor_height_habitat=float(floor_height),
        height_tolerance=float(height_tolerance),
        reference_island_index=reference_island,
        included_island_indices=included_islands,
    )


def write_ros_occupancy_map(root: Path, occupancy: RosOccupancyMap) -> None:
    root.mkdir(parents=True, exist_ok=True)
    Image.fromarray(occupancy.image, mode="L").save(root / "map.pgm")
    ox, oy, yaw = occupancy.origin
    (root / "map.yaml").write_text(
        "\n".join([
            "image: map.pgm",
            f"resolution: {occupancy.resolution:.9g}",
            f"origin: [{ox:.9g}, {oy:.9g}, {yaw:.9g}]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
            "",
        ]),
        encoding="ascii",
    )
