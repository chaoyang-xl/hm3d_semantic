import numpy as np

from hm3d_reconstruction.exploration import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    simulate_lidar_exploration,
)
from hm3d_reconstruction.occupancy import RosOccupancyMap


def test_lidar_exploration_preserves_unknown_cells():
    ground_image = np.zeros((7, 7), dtype=np.uint8)
    ground_image[1:6, 1:6] = FREE
    ground_truth = RosOccupancyMap(
        image=ground_image,
        resolution=1.0,
        origin=(0.0, 0.0, 0.0),
        floor_height_habitat=0.0,
        height_tolerance=0.2,
        reference_island_index=0,
        included_island_indices=(0,),
    )
    explored, report = simulate_lidar_exploration(
        ground_truth,
        positions_xy=np.array([[3.5, 3.5]]),
        ray_count=4,
        max_range_m=10.0,
    )
    assert explored.image[3, 3] == FREE
    assert explored.image[3, 0] == OCCUPIED
    assert explored.image[0, 3] == OCCUPIED
    assert explored.image[2, 2] == UNKNOWN
    assert report["unique_scan_positions"] == 1
    assert 0.0 < report["explored_ratio"] < 1.0
