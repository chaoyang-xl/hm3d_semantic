import json

import numpy as np
import pytest

from conftest import MockSource
from hm3d_reconstruction.dataset import read_traj_gt
from hm3d_reconstruction.exporter import export_dataset
from hm3d_reconstruction.validator import validate_dataset


def test_mock_export_roundtrip(config):
    output=export_dataset(config, source_factory=MockSource)
    assert read_traj_gt(output/"traj_gt.txt").shape == (3,4,4)
    assert not output.with_name("output.partial").exists()
    report=json.loads((output/"export_report.json").read_text())
    assert report["semantic_metadata_coverage"] == 1
    result=validate_dataset(output, sample_count=3, strict=True)
    assert result.valid, result.errors


def test_refuses_nonempty_output(config):
    config.output.mkdir(); (config.output/"keep").write_text("x")
    with pytest.raises(FileExistsError):
        export_dataset(config, source_factory=MockSource)


def test_failure_keeps_partial(config):
    class Broken(MockSource):
        def frames(self):
            yield next(super().frames())
            raise RuntimeError("injected")
    with pytest.raises(RuntimeError):
        export_dataset(config, source_factory=Broken)
    partial=config.output.with_name("output.partial")
    assert (partial/"failure_report.json").is_file()
    assert not config.output.exists()


def test_traj_rejects_bad_shape(tmp_path):
    path=tmp_path/"traj_gt.txt"; np.savetxt(path,np.ones((3,4)))
    with pytest.raises(ValueError):
        read_traj_gt(path)

