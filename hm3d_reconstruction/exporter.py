from __future__ import annotations

import shutil
import time
import traceback
from pathlib import Path
from typing import Callable

import numpy as np

from .config import CaptureConfig
from .coordinate import habitat_c2w_to_map_z_up
from .dataset import (
    save_depth, save_rgb, save_semantic, write_json, write_traj_gt,
)
from .depth import depth_meters_to_uint16_mm
from .intrinsics import compute_pinhole_intrinsics
from .exploration import simulate_lidar_exploration
from .occupancy import write_ros_occupancy_map
from .simulator import HabitatCapture
from .visualization import write_previews, write_trajectory


def _prepare(output: Path, overwrite: bool) -> Path:
    partial = output.with_name(output.name + ".partial")
    if output.exists() and any(output.iterdir()) and not overwrite:
        raise FileExistsError(f"non-empty output exists: {output}")
    if partial.exists() and not overwrite:
        raise FileExistsError(f"partial output exists: {partial}")
    if partial.exists():
        shutil.rmtree(partial)
    if output.exists() and not any(output.iterdir()):
        output.rmdir()
    partial.mkdir(parents=True)
    for name in ("results", "pose_gt", "semantic"):
        (partial/name).mkdir()
    return partial


def _publish(partial: Path, output: Path, overwrite: bool) -> None:
    backup = output.with_name(output.name + ".previous")
    if backup.exists():
        shutil.rmtree(backup)
    if output.exists():
        if not overwrite:
            raise FileExistsError(output)
        output.rename(backup)
    try:
        partial.rename(output)
    except Exception:
        if backup.exists():
            backup.rename(output)
        raise
    if backup.exists():
        shutil.rmtree(backup)


def export_dataset(
    config: CaptureConfig,
    source_factory: Callable = HabitatCapture,
) -> Path:
    config.validate()
    output = config.output.resolve()
    partial = _prepare(output, config.overwrite)
    source, poses, trajectory = None, [], []
    invalid_depth = total_depth = semantic_pixels = matched_pixels = 0
    started = time.monotonic()
    try:
        intrinsics = compute_pinhole_intrinsics(
            config.width, config.height, config.hfov_deg
        )
        write_json(partial/"cam_params.json", {"camera": intrinsics.as_camera_dict()})
        source = source_factory(config)
        write_json(partial/"semantic_metadata.json", source.semantic_metadata)
        known_ids = {int(key) for key in source.semantic_metadata["instances"]}
        for index, frame in enumerate(source.frames()):
            if index >= config.frames:
                raise RuntimeError("source produced too many frames")
            depth = depth_meters_to_uint16_mm(
                frame.depth_m, config.min_depth_m, config.max_depth_m
            )
            save_rgb(partial/"results"/f"frame{index:06d}.jpg", frame.rgb)
            save_depth(partial/"results"/f"depth{index:06d}.png", depth)
            np.savetxt(partial/"pose_gt"/f"{index:06d}.txt", frame.pose_gt, fmt="%.9g")
            if config.save_semantic:
                if frame.semantic is None:
                    raise RuntimeError("semantic observation missing")
                save_semantic(
                    partial/"semantic"/f"semantic{index:06d}.png", frame.semantic
                )
                semantic_pixels += frame.semantic.size
                matched_pixels += int(np.isin(frame.semantic, list(known_ids)).sum())
            invalid_depth += int((depth == 0).sum())
            total_depth += depth.size
            poses.append(frame.pose_gt)
            trajectory.append(frame.trajectory)
        if not poses:
            raise RuntimeError("capture produced no recorded frames")
        if config.trajectory_mode != "interactive" and len(poses) != config.frames:
            raise RuntimeError(f"wrote {len(poses)} frames, expected {config.frames}")
        write_traj_gt(partial/"traj_gt.txt", poses)
        write_traj_gt(
            partial/"traj.txt",
            [habitat_c2w_to_map_z_up(pose) for pose in poses],
        )
        write_json(partial/"trajectory.json", {"frames": trajectory})
        ros_map_metadata = None
        if config.export_ros_map:
            positions = np.asarray(
                [frame["agent_position"] for frame in trajectory],
                dtype=np.float64,
            )
            floor_height = (
                float(config.map_floor_height_m)
                if config.map_floor_height_m is not None
                else float(np.median(positions[:, 1]))
            )
            occupancy = source.build_ros_occupancy_map(
                resolution=config.map_resolution_m,
                floor_height=floor_height,
                height_tolerance=config.map_height_tolerance_m,
                reference_position=positions[0],
                min_island_area=config.map_min_island_area_m2,
            )
            exploration_metadata = {"map_mode": "ground_truth"}
            if config.map_mode == "explored":
                positions_xy = np.column_stack((
                    positions[:, 0], -positions[:, 2],
                ))
                occupancy, exploration_metadata = simulate_lidar_exploration(
                    occupancy,
                    positions_xy=positions_xy,
                    ray_count=config.map_ray_count,
                    max_range_m=config.map_max_range_m,
                )
            write_ros_occupancy_map(partial, occupancy)
            ros_map_metadata = occupancy.metadata()
            ros_map_metadata.update(exploration_metadata)
        metadata = {
            "schema_version": 1,
            "dataset_type": "hm3d_semantic_gt_rgbd",
            "frame_count": len(poses),
            "depth_unit": "millimeter",
            "pose_convention": "T_world_camera_gt",
            "camera_convention": "opencv_optical",
            "world_convention": "habitat_y_up",
            "traj_world_convention": "map_z_up",
            "traj_from_gt": (
                "x_map=x_habitat,y_map=-z_habitat,z_map=y_habitat"
            ),
            "scene": str(config.scene.resolve()),
            "trajectory_mode": config.trajectory_mode,
            "capture_stop_reason": getattr(source, "stop_reason", "frame_limit"),
            "semantic_enabled": config.save_semantic,
            "ros_map_enabled": config.export_ros_map,
        }
        if ros_map_metadata is not None:
            metadata["ros_map"] = ros_map_metadata
        write_json(partial/"metadata.json", metadata)
        positions = np.asarray([f["agent_position"] for f in trajectory])
        steps = np.linalg.norm(np.diff(positions, axis=0), axis=1) if len(positions)>1 else np.array([0.])
        report = {
            "schema_version": 1,
            "requested_frames": config.frames,
            "output_frames": len(poses),
            "invalid_depth_ratio": invalid_depth/max(total_depth, 1),
            "semantic_metadata_coverage": matched_pixels/max(semantic_pixels, 1),
            "collision_count": source.collision_count,
            "navigable_area_m2": source.navigable_area,
            "trajectory_length_m": float(steps.sum()),
            "mean_frame_translation_m": float(steps.mean()),
            "max_frame_translation_m": float(steps.max()),
            "turn_in_place_frames": sum(f["action"]=="turn_in_place" for f in trajectory),
            "move_forward_frames": sum(f["action"]=="move_forward" for f in trajectory),
            "move_backward_frames": sum(f["action"]=="move_backward" for f in trajectory),
            "interactive_turn_frames": sum(
                f["action"] in {"turn_left", "turn_right"} for f in trajectory
            ),
            "elapsed_seconds": time.monotonic()-started,
        }
        if ros_map_metadata is not None:
            report["ros_map"] = ros_map_metadata
        write_json(partial/"export_report.json", report)
        if config.preview:
            indices=np.linspace(0,len(poses)-1,min(8,len(poses)),dtype=int).tolist()
            write_previews(partial,indices,config.save_semantic)
            write_trajectory(positions,partial/"preview"/"trajectory_topdown.png")
        from .validator import validate_dataset
        validation = validate_dataset(partial, sample_count=config.frames, strict=True)
        if not validation.valid:
            raise RuntimeError("pre-publication validation failed: " + "; ".join(validation.errors))
        keep_partial = bool(getattr(source, "keep_partial", False))
        source.close()
        source = None
        if keep_partial:
            return partial
        _publish(partial, output, config.overwrite)
        return output
    except Exception as exc:
        write_json(partial/"failure_report.json", {
            "error_type": type(exc).__name__, "error": str(exc),
            "frames_written": len(poses), "traceback": traceback.format_exc(),
            "config": config.serializable(),
        })
        raise
    finally:
        if source is not None:
            source.close()

