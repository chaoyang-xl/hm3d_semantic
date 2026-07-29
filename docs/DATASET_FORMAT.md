# Dataset Format

`traj_gt.txt` stacks N camera-to-world matrices into 4N rows. Individual
matrices are under `pose_gt/`. RGB JPEG and uint16 millimeter depth PNG files
are under `results/`; raw uint16 semantic instance IDs are under `semantic/`.
`semantic_metadata.json` maps instance IDs to categories and regions.
`metadata.json` declares schema, units, and conventions.

