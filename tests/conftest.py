from pathlib import Path

import numpy as np
import pytest

from hm3d_reconstruction.config import CaptureConfig
from hm3d_reconstruction.simulator import CapturedFrame


@pytest.fixture
def config(tmp_path: Path) -> CaptureConfig:
    scene, dataset = tmp_path/"scene.basis.glb", tmp_path/"dataset.json"
    scene.write_bytes(b"mock"); dataset.write_text("{}")
    return CaptureConfig(scene, dataset, tmp_path/"output", frames=3, width=8, height=6)


class MockSource:
    navigable_area = 12.0
    collision_count = 0
    semantic_metadata = {"instances": {"1": {"instance_id": 1, "category_name": "chair", "category_index": 2, "region_id": "0", "semantic_object_id": "chair_1"}}}

    def __init__(self, config):
        self.config = config

    def frames(self):
        for index in range(self.config.frames):
            pose = np.eye(4); pose[0,3] = index*0.1
            yield CapturedFrame(
                np.full((self.config.height,self.config.width,4), 100, np.uint8),
                np.full((self.config.height,self.config.width), 1.5, np.float32),
                np.ones((self.config.height,self.config.width), np.uint32),
                pose,
                {"frame_index": index, "agent_position": [index*0.1,0,0], "agent_rotation_xyzw": [0,0,0,1], "sensor_position": [index*0.1,.88,0], "sensor_rotation_xyzw": [0,0,0,1], "action": "move_forward", "collision": False},
            )

    def close(self):
        pass

