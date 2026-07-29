# HM3D-Semantic Reconstruction

Independent Habitat-Sim capture tools for synchronized RGB, metric depth,
semantic instance IDs, and ground-truth OpenCV-optical camera poses. This
stage contains no ROS, SLAM, detection, or tracking dependency.

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

