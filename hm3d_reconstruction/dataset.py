from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from PIL import Image

from .coordinate import validate_rigid_transform


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_traj_gt(path: Path, poses: Iterable[np.ndarray]) -> None:
    checked = [validate_rigid_transform(pose) for pose in poses]
    if not checked:
        raise ValueError("trajectory is empty")
    np.savetxt(path, np.vstack(checked), fmt="%.9g")


def read_traj_gt(path: Path) -> np.ndarray:
    values = np.loadtxt(path, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 4 or values.shape[0] % 4:
        raise ValueError("traj_gt.txt must have shape (4N, 4)")
    poses = values.reshape(-1, 4, 4)
    for index, pose in enumerate(poses):
        validate_rigid_transform(pose, f"pose {index}")
    return poses


def save_rgb(path: Path, rgb: np.ndarray) -> None:
    value = np.asarray(rgb)
    if value.dtype != np.uint8 or value.ndim != 3 or value.shape[2] not in (3, 4):
        raise ValueError("RGB must be uint8 HxWx3/4")
    Image.fromarray(value[:, :, :3]).save(path, quality=95)


def save_depth(path: Path, depth: np.ndarray) -> None:
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError("depth must be uint16 HxW")
    Image.fromarray(depth).save(path)


def save_semantic(path: Path, semantic: np.ndarray) -> None:
    value = np.asarray(semantic)
    if value.ndim != 2 or value.min(initial=0) < 0 or value.max(initial=0) > 65535:
        raise ValueError("semantic IDs must fit uint16")
    Image.fromarray(value.astype(np.uint16)).save(path)


def indexed(directory: Path, prefix: str, suffix: str) -> dict[int, Path]:
    result = {}
    if not directory.is_dir():
        return result
    for path in directory.iterdir():
        token = path.name[len(prefix):-len(suffix)] if path.name.startswith(prefix) and path.name.endswith(suffix) else ""
        if token.isdigit():
            result[int(token)] = path
    return result

