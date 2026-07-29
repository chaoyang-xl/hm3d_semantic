import numpy as np

from hm3d_reconstruction.simulator import densify_path
from hm3d_reconstruction.trajectory import yaw_facing_habitat_direction


def test_densify_path_bounds_translation():
    points=densify_path([np.array([0,0,0]),np.array([0,0,-.25])],.1)
    previous=np.array([0,0,0])
    for point in points:
        assert np.linalg.norm(point-previous) <= .100001
        previous=point


def test_habitat_forward_yaw_zero():
    assert yaw_facing_habitat_direction(np.array([0,-1])) == 0

