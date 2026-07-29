# HM3D-Semantic Reconstruction

Independent Habitat-Sim capture tools for synchronized RGB, metric depth,
semantic instance IDs, and ground-truth OpenCV-optical camera poses. This
package contains no ROS, SLAM, detection, or tracking dependency.

## Install

```bash
conda env create -f environment.yml
conda activate hm3d_reconstruction
```

## Inspect, Export, Validate

```bash
python scripts/inspect_scene.py \
  --scene /data/hm3d/00800/scene.basis.glb \
  --scene-dataset-config /data/hm3d/hm3d_annotated_basis.scene_dataset_config.json \
  --output /tmp/hm3d_scene_check

python scripts/export_hm3d_dataset.py \
  --scene /data/hm3d/00800/scene.basis.glb \
  --scene-dataset-config /data/hm3d/hm3d_annotated_basis.scene_dataset_config.json \
  --output /data/exports/scene_00800 --frames 100 \
  --width 640 --height 480 --trajectory-mode waypoint \
  --save-semantic --preview

python scripts/validate_dataset.py \
  --data-root /data/exports/scene_00800 \
  --sample-count 50 --strict --write-preview
```

Waypoint capture rotates in place until aligned, then advances through
`PathFinder.try_step()`. It does not move sideways or teleport across walls.
See `docs/` for coordinate, dataset, trajectory, and setup contracts.

## Interactive Capture

Interactive capture writes the same RGB-D-Semantic and GT pose dataset:

```bash
python scripts/export_hm3d_dataset.py \
  --scene /data/hm3d/00800/scene.basis.glb \
  --scene-dataset-config /data/hm3d/hm3d_annotated_basis.scene_dataset_config.json \
  --output /data/exports/scene_00800_manual --frames 1000 \
  --width 640 --height 480 --display-scale 1.5 \
  --trajectory-mode interactive \
  --save-semantic --preview
```

The window starts paused. Use `R` to record, `P` to pause, `W/S` to move,
`A/D` to turn, `Q` to save, and `Esc` to retain a complete partial dataset.
Movement before the first `R` press is preview-only and is not written to disk.

### Output Path

`--output` is required and has no implicit default. An absolute path is used as
written. A relative path is resolved from the directory where the command is
started. For example, when running from this repository,
`--output outputs/00804_manual` saves the completed dataset to:

```text
/home/weiyu/vscode_workspace/hd3d_semantic/outputs/00804_manual
```

Capture first writes beside it as `outputs/00804_manual.partial`. `Q` validates
and renames it to the final path. After at least one recorded frame, `Esc`
validates a complete dataset but keeps the `.partial` directory name.

### Capture Parameters

| Parameter | Required/default | Meaning |
| --- | --- | --- |
| `--scene` | required | HM3D `.basis.glb` render asset |
| `--scene-dataset-config` | required | Habitat scene dataset configuration JSON |
| `--output` | required | Final dataset directory; no default |
| `--frames` | `100` | Exact waypoint/replay frames; maximum interactive recorded frames |
| `--width`, `--height` | `640`, `480` | RGB, depth, and semantic image resolution in pixels |
| `--hfov-deg` | `79` | Horizontal camera field of view in degrees |
| `--sensor-height` | `0.88` | Camera height above the agent base in metres |
| `--display-scale` | `1.5` | Interactive window scale only; saved resolution is unchanged |
| `--min-depth-m`, `--max-depth-m` | `0.05`, `10` | Valid depth interval in metres; other pixels become zero |
| `--trajectory-mode` | `waypoint` | `waypoint`, `interactive`, or `replay` |
| `--trajectory-file` | unset | Position/quaternion JSON required by `replay` only |
| `--forward-step` | `0.10` | Forward/backward distance per action in metres |
| `--turn-angle-deg` | `5` | Rotation per action in degrees |
| `--alignment-tolerance-deg` | `10` | Waypoint-only heading tolerance before moving |
| `--seed` | `42` | Reproducible random start and waypoint seed |
| `--save-semantic` | enabled | Save Habitat GT instance IDs; disable with `--no-save-semantic` |
| `--preview` | disabled | Save sample mosaics and top-down trajectory image |
| `--overwrite` | disabled | Transactionally replace an existing output directory |

## Optional Predicted Semantics

Scenes without HM3D semantic annotations can be exported with
`--no-save-semantic`. The exporter writes both the canonical `traj_gt.txt` and
an identical Replica-compatible `traj.txt`, so the resulting RGB-D dataset can
be processed by an external YOLO-World + MobileSAM pipeline without changing
the GT camera poses.

Predicted labels and masks are model output, not HM3D ground-truth semantics.
Keep model weights, generated detections, and point clouds outside this
repository.
