import json
from dataclasses import replace

import numpy as np
from PIL import Image

from conftest import MockSource
from hm3d_reconstruction.exporter import export_dataset
from hm3d_reconstruction.occupancy import (
    RosOccupancyMap,
    occupancy_from_pathfinder,
    write_ros_occupancy_map,
)


class FakePathfinder:
    def get_bounds(self):
        return np.array([1.0, -1.0, 10.0]), np.array([3.0, 1.0, 11.5])

    def get_island(self, reference):
        assert np.asarray(reference).shape == (3,)
        return 4

    def get_topdown_island_view(self, resolution, height, eps):
        assert resolution == 0.5
        assert height == 0.25
        assert eps == 0.1
        return np.array([
            [-1, 4, 4, -1],
            [-1, 4, 7, -1],
            [-1, 4, 4, -1],
        ], dtype=np.int32)

    def island_area(self, island_index):
        return {4: 2.0, 7: 0.2}[island_index]


def test_pathfinder_map_uses_ros_z_up_coordinates():
    occupancy = occupancy_from_pathfinder(
        FakePathfinder(), 0.5, 0.25, 0.1, np.array([1.75, 0.25, 10.25])
    )
    np.testing.assert_array_equal(
        occupancy.image,
        np.array([
            [0, 254, 254, 0],
            [0, 254, 0, 0],
            [0, 254, 254, 0],
        ], dtype=np.uint8),
    )
    # Habitat cell (row=0, column=1) has x=1.75, z=10.25.
    assert occupancy.world_to_pixel(1.75, -10.25) == (0, 1)
    # Habitat cell (row=2, column=2) has x=2.25, z=11.25.
    assert occupancy.world_to_pixel(2.25, -11.25) == (2, 2)


def test_write_ros_map_server_files(tmp_path):
    occupancy = occupancy_from_pathfinder(
        FakePathfinder(), 0.5, 0.25, 0.1, np.array([1.75, 0.25, 10.25])
    )
    write_ros_occupancy_map(tmp_path, occupancy)
    np.testing.assert_array_equal(
        np.asarray(Image.open(tmp_path / "map.pgm")), occupancy.image
    )
    yaml_text = (tmp_path / "map.yaml").read_text(encoding="ascii")
    assert "image: map.pgm" in yaml_text
    assert "resolution: 0.5" in yaml_text
    assert "origin: [1, -11.5, 0]" in yaml_text


def test_dataset_export_can_include_ros_map(config):
    class MapSource(MockSource):
        def build_ros_occupancy_map(
            self, resolution, floor_height, height_tolerance,
            reference_position, min_island_area,
        ):
            assert floor_height == 0.0
            return RosOccupancyMap(
                image=np.array([[0, 254], [254, 254]], dtype=np.uint8),
                resolution=resolution,
                origin=(0.0, -0.1, 0.0),
                floor_height_habitat=floor_height,
                height_tolerance=height_tolerance,
                reference_island_index=0,
                included_island_indices=(0,),
            )

    mapped = replace(config, export_ros_map=True)
    output = export_dataset(mapped, source_factory=MapSource)
    assert (output / "map.pgm").is_file()
    assert (output / "map.yaml").is_file()
    metadata = json.loads((output / "metadata.json").read_text())
    assert metadata["ros_map_enabled"] is True
    assert metadata["ros_map"]["coordinate_conversion"] == (
        "x_map=x_habitat,y_map=-z_habitat"
    )
