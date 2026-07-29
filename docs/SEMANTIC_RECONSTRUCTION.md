# Semantic Reconstruction

The first stage exports the synchronized ground-truth inputs required for
reconstruction. The second stage will back-project valid depth, transform by
`T_world_camera_gt`, aggregate stable instance IDs, voxel-downsample points,
and produce RGB, semantic, and per-instance PLY/NPZ outputs.

