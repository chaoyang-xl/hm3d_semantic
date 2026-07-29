from __future__ import annotations

import numpy as np

C_HABITAT_FROM_OPENCV = np.diag([1.0, -1.0, -1.0, 1.0])


def rigid_transform_errors(matrix: np.ndarray, atol: float = 1e-5) -> list[str]:
    value = np.asarray(matrix, dtype=np.float64)
    if value.shape != (4, 4):
        return [f"expected (4, 4), got {value.shape}"]
    errors = []
    if not np.isfinite(value).all():
        return ["matrix contains non-finite values"]
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=atol):
        errors.append("rotation is not orthogonal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=atol):
        errors.append("rotation determinant is not 1")
    if not np.allclose(value[3], [0, 0, 0, 1], atol=atol):
        errors.append("last row is not [0, 0, 0, 1]")
    return errors


def validate_rigid_transform(matrix: np.ndarray, name: str = "pose") -> np.ndarray:
    value = np.asarray(matrix, dtype=np.float64)
    errors = rigid_transform_errors(value)
    if errors:
        raise ValueError(f"{name} invalid: {'; '.join(errors)}")
    return value


def habitat_sensor_pose_to_opencv_c2w(pose: np.ndarray) -> np.ndarray:
    habitat_pose = validate_rigid_transform(pose, "Habitat sensor pose")
    return validate_rigid_transform(
        habitat_pose @ C_HABITAT_FROM_OPENCV, "OpenCV camera pose"
    )


def quaternion_xyzw_to_matrix(position: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    p = np.asarray(position, dtype=np.float64)
    q = np.asarray(quaternion, dtype=np.float64)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError("position/quaternion shape or values invalid")
    q /= np.linalg.norm(q)
    x, y, z, w = q
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    result[:3, 3] = p
    return result

