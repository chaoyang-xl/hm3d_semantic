from __future__ import annotations

from typing import Any


COLOR_UUID = "color"
DEPTH_UUID = "depth"
SEMANTIC_UUID = "semantic"


def camera_sensor_spec(
    habitat_sim: Any, uuid: str, sensor_type: Any, config: Any
) -> Any:
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid = uuid
    spec.sensor_type = sensor_type
    spec.resolution = [config.height, config.width]
    spec.position = [0.0, config.sensor_height, 0.0]
    spec.orientation = [0.0, 0.0, 0.0]
    spec.hfov = config.hfov_deg
    if uuid == DEPTH_UUID:
        spec.near = config.min_depth_m
        spec.far = config.max_depth_m
    return spec

