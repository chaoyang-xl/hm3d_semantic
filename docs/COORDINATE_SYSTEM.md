# Coordinate System

Habitat world is right-handed with Y up. Exported camera-local coordinates are
OpenCV optical: X right, Y down, Z forward. Every saved pose is camera-to-world:

```text
P_world = T_world_camera_gt P_camera
T_world_camera_gt = T_world_sensor_habitat diag(1,-1,-1,1)
```

Matrices are checked for finite values, orthogonal rotation, determinant one,
and homogeneous last row.

