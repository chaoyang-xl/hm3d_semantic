from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image

from .coordinate import habitat_c2w_to_map_z_up, rigid_transform_errors
from .dataset import indexed, read_traj_gt
from .visualization import write_previews, write_trajectory


@dataclass
class ValidationResult:
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    checks: dict = field(default_factory=dict)

    @property
    def valid(self) -> bool:
        return not self.errors


def validate_dataset(
    root: Path, sample_count: int = 50, strict: bool = False,
    write_preview: bool = False,
) -> ValidationResult:
    result = ValidationResult()
    try:
        camera = json.loads((root/"cam_params.json").read_text())["camera"]
        metadata = json.loads((root/"metadata.json").read_text())
        semantic_metadata = json.loads((root/"semantic_metadata.json").read_text())
        poses = read_traj_gt(root/"traj_gt.txt")
        replica_poses = read_traj_gt(root/"traj.txt")
    except Exception as exc:
        result.errors.append(str(exc))
        return result
    expected_replica_poses = np.asarray([
        habitat_c2w_to_map_z_up(pose) for pose in poses
    ])
    replica_compatible = replica_poses.shape == poses.shape and np.allclose(
        replica_poses, expected_replica_poses, atol=1e-9
    )
    if not replica_compatible:
        result.errors.append(
            "traj.txt is not the Z-up map transform of traj_gt.txt"
        )
    required = {"w","h","fx","fy","cx","cy","scale"}
    if not required.issubset(camera):
        result.errors.append("cam_params fields missing")
        return result
    files = {
        "rgb": indexed(root/"results", "frame", ".jpg"),
        "depth": indexed(root/"results", "depth", ".png"),
        "pose": indexed(root/"pose_gt", "", ".txt"),
    }
    if metadata.get("semantic_enabled"):
        files["semantic"] = indexed(root/"semantic", "semantic", ".png")
    count = len(poses)
    for label, mapping in files.items():
        if sorted(mapping) != list(range(count)):
            result.errors.append(f"{label} indices are not contiguous")
        if len(mapping) != count:
            result.errors.append(f"{label} count differs from trajectory")
    if metadata.get("frame_count") != count:
        result.errors.append("metadata frame_count mismatch")
    indices = np.linspace(0, count-1, min(max(sample_count,1),count), dtype=int)
    known = {int(key) for key in semantic_metadata.get("instances", {})}
    observed = set()
    for index in indices:
        rgb, depth = np.asarray(Image.open(files["rgb"][index])), np.asarray(Image.open(files["depth"][index]))
        if rgb.shape[:2] != (camera["h"], camera["w"]):
            result.errors.append(f"RGB shape mismatch at {index}")
        if depth.shape != (camera["h"], camera["w"]) or depth.dtype != np.uint16:
            result.errors.append(f"depth invalid at {index}")
        disk_pose = np.loadtxt(files["pose"][index])
        if rigid_transform_errors(disk_pose) or not np.allclose(disk_pose, poses[index], atol=1e-6):
            result.errors.append(f"pose invalid at {index}")
        y, x = np.nonzero(depth)
        if len(x):
            z = depth[y[:32],x[:32]]/camera["scale"]
            points = np.stack([(x[:32]-camera["cx"])*z/camera["fx"], (y[:32]-camera["cy"])*z/camera["fy"], z, np.ones_like(z)])
            if not np.isfinite(poses[index] @ points).all() or not (z>0).all():
                result.errors.append(f"backprojection invalid at {index}")
        if "semantic" in files:
            semantic = np.asarray(Image.open(files["semantic"][index]))
            if semantic.shape != depth.shape:
                result.errors.append(f"semantic shape mismatch at {index}")
            observed.update(int(v) for v in np.unique(semantic))
    unmatched = sorted(observed-known)
    if unmatched:
        (result.errors if strict else result.warnings).append(f"unmatched semantic IDs: {unmatched[:20]}")
    translations = np.linalg.norm(np.diff(poses[:,:3,3], axis=0), axis=1) if count>1 else np.array([0.])
    if translations.max() > 1.0:
        (result.errors if strict else result.warnings).append("abnormal adjacent pose translation")
    result.checks = {
        "frame_count": count,
        "sampled_frames": len(indices),
        "resolution": [camera["w"], camera["h"]],
        "replica_trajectory_compatible": bool(replica_compatible),
        "unmatched_semantic_ids": unmatched,
        "max_translation_m": float(translations.max()),
    }
    if write_preview and not result.errors:
        write_previews(root,indices[:8].tolist(),metadata.get("semantic_enabled",False))
        trajectory=json.loads((root/"trajectory.json").read_text())["frames"]
        write_trajectory(np.asarray([f["agent_position"] for f in trajectory]),root/"preview"/"trajectory_topdown.png")
    return result

