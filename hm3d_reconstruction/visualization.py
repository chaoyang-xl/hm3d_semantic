from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def _tile(images: list[Image.Image], output: Path) -> None:
    copies = []
    for image in images:
        value = image.convert("RGB")
        value.thumbnail((240, 180))
        copies.append(value)
    if not copies:
        return
    width = max(v.width for v in copies)
    height = max(v.height for v in copies)
    canvas = Image.new("RGB", (width * 4, height * ((len(copies)+3)//4)), "white")
    for index, value in enumerate(copies):
        canvas.paste(value, ((index % 4)*width, (index // 4)*height))
    canvas.save(output, quality=92)


def depth_color(depth: np.ndarray) -> Image.Image:
    value = np.asarray(depth)
    valid = value > 0
    normalized = np.zeros(value.shape, float)
    if valid.any():
        low, high = np.percentile(value[valid], [2, 98])
        normalized[valid] = np.clip((value[valid]-low)/(max(high, low+1)-low), 0, 1)
    rgb = np.stack([normalized, 1-np.abs(2*normalized-1), 1-normalized], axis=-1)
    rgb[~valid] = 0
    return Image.fromarray((rgb*255).astype(np.uint8))


def semantic_color(ids: np.ndarray) -> Image.Image:
    value = np.asarray(ids, dtype=np.uint64)
    rgb = np.stack(
        [(value*37+17)%251, (value*73+29)%253, (value*109+43)%255], axis=-1
    ).astype(np.uint8)
    rgb[value == 0] = 0
    return Image.fromarray(rgb)


def write_previews(root: Path, indices: list[int], semantic: bool) -> None:
    preview = root/"preview"
    preview.mkdir(exist_ok=True)
    _tile([Image.open(root/"results"/f"frame{i:06d}.jpg") for i in indices], preview/"rgb_samples.jpg")
    _tile([depth_color(np.asarray(Image.open(root/"results"/f"depth{i:06d}.png"))) for i in indices], preview/"depth_samples.jpg")
    if semantic:
        _tile([semantic_color(np.asarray(Image.open(root/"semantic"/f"semantic{i:06d}.png"))) for i in indices], preview/"semantic_samples.jpg")


def write_trajectory(positions: np.ndarray, output: Path) -> None:
    xz = np.asarray(positions)[:, [0, 2]]
    low = xz.min(0)
    span = np.maximum(xz.max(0)-low, 1e-6)
    points = (xz-low)*(736/max(span))+32
    points[:, 1] = 800-points[:, 1]
    image = Image.new("RGB", (800, 800), "white")
    if len(points) > 1:
        ImageDraw.Draw(image).line([tuple(p) for p in points], fill=(20,90,190), width=3)
    image.save(output)
