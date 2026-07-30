import math

import numpy as np
import pytest

from hm3d_reconstruction.coordinate import (
    C_HABITAT_FROM_OPENCV, C_MAP_FROM_HABITAT,
    habitat_c2w_to_map_z_up, habitat_sensor_pose_to_opencv_c2w,
    rigid_transform_errors,
)
from hm3d_reconstruction.depth import depth_meters_to_uint16_mm
from hm3d_reconstruction.intrinsics import compute_pinhole_intrinsics
from hm3d_reconstruction.simulator import (
    TkInteractiveWindow, interactive_displacement,
)
from hm3d_reconstruction.trajectory import turn_toward


def test_intrinsics():
    value = compute_pinhole_intrinsics(640,480,90)
    assert value.fx == pytest.approx(320)
    assert (value.cx,value.cy) == pytest.approx((319.5,239.5))


def test_coordinate_conversion():
    pose = habitat_sensor_pose_to_opencv_c2w(np.eye(4))
    np.testing.assert_allclose(pose, C_HABITAT_FROM_OPENCV)
    assert not rigid_transform_errors(pose)


def test_habitat_y_up_pose_converts_to_map_z_up():
    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 3.0]
    converted = habitat_c2w_to_map_z_up(pose)
    np.testing.assert_allclose(converted, C_MAP_FROM_HABITAT @ pose)
    np.testing.assert_allclose(converted[:3, 3], [1.0, -3.0, 2.0])
    assert np.linalg.det(converted[:3, :3]) == pytest.approx(1.0)


def test_invalid_rigid_matrix():
    value=np.eye(4); value[0,0]=2
    assert rigid_transform_errors(value)


def test_depth_conversion():
    value=np.array([[1.2346,np.nan,np.inf,0,-1,11,.01]])
    result=depth_meters_to_uint16_mm(value,.05,10)
    np.testing.assert_array_equal(result, [[1235,0,0,0,0,0,0]])
    assert result.dtype == np.uint16


def test_turn_is_bounded():
    assert turn_toward(0,math.pi,math.radians(5)) == pytest.approx(-math.radians(5))


def test_interactive_displacement_follows_habitat_heading():
    np.testing.assert_allclose(interactive_displacement(0.0, 0.1), [0, 0, -0.1])
    np.testing.assert_allclose(
        interactive_displacement(-math.pi / 2, 0.1), [0.1, 0, 0], atol=1e-9
    )


def test_interactive_key_map():
    assert TkInteractiveWindow.KEY_COMMANDS["w"] == "forward"
    assert TkInteractiveWindow.KEY_COMMANDS["escape"] == "abort"

