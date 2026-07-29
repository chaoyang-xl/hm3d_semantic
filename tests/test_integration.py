import importlib.util
import os
from pathlib import Path

import pytest

from hm3d_reconstruction.config import CaptureConfig
from hm3d_reconstruction.exporter import export_dataset
from hm3d_reconstruction.validator import validate_dataset


@pytest.mark.integration
def test_real_scene(tmp_path):
    if importlib.util.find_spec("habitat_sim") is None:
        pytest.skip("Habitat-Sim unavailable")
    scene=os.getenv("HM3D_TEST_SCENE"); dataset=os.getenv("HM3D_TEST_DATASET_CONFIG")
    if not scene or not dataset:
        pytest.skip("HM3D test paths unavailable")
    config=CaptureConfig(Path(scene),Path(dataset),tmp_path/"real",frames=10,width=64,height=48)
    output=export_dataset(config)
    assert validate_dataset(output,10,True).valid

