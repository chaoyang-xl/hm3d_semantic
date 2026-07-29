from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_quaternion_xyzw(yaw: float) -> np.ndarray:
    return np.array([0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2)])


def yaw_from_quaternion_xyzw(q: np.ndarray) -> float:
    x, y, z, w = np.asarray(q, dtype=np.float64)
    return math.atan2(2*(w*y+x*z), 1-2*(y*y+z*z))


def yaw_facing_habitat_direction(direction_xz: np.ndarray) -> float:
    direction = np.asarray(direction_xz, dtype=np.float64)
    return math.atan2(-direction[0], -direction[1])


def turn_toward(current: float, target: float, max_step: float) -> float:
    return current + float(np.clip(wrap_angle(target-current), -max_step, max_step))


def load_replay(path: Path) -> list[tuple[np.ndarray, np.ndarray]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    entries = payload.get("frames", payload) if isinstance(payload, dict) else payload
    result = []
    for index, item in enumerate(entries):
        position = np.asarray(item.get("agent_position", item.get("position")), dtype=float)
        rotation = np.asarray(item.get("agent_rotation_xyzw", item.get("rotation_xyzw")), dtype=float)
        if position.shape != (3,) or rotation.shape != (4,) or not np.isfinite(position).all() or not np.isfinite(rotation).all():
            raise ValueError(f"invalid replay state {index}")
        rotation /= np.linalg.norm(rotation)
        result.append((position, rotation))
    if not result:
        raise ValueError("replay is empty")
    return result

