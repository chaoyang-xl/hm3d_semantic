import json
from dataclasses import replace

import numpy as np
import pytest

from conftest import MockSource
from hm3d_reconstruction.dataset import read_traj_gt
from hm3d_reconstruction.exporter import export_dataset
from hm3d_reconstruction.validator import validate_dataset


def test_mock_export_roundtrip(config):
    output=export_dataset(config, source_factory=MockSource)
    poses = read_traj_gt(output/"traj_gt.txt")
    replica_poses = read_traj_gt(output/"traj.txt")
    assert poses.shape == (3,4,4)
    np.testing.assert_allclose(replica_poses, poses)
    assert not output.with_name("output.partial").exists()
    report=json.loads((output/"export_report.json").read_text())
    assert report["semantic_metadata_coverage"] == 1
    result=validate_dataset(output, sample_count=3, strict=True)
    assert result.valid, result.errors


def test_validator_rejects_mismatched_replica_trajectory(config):
    output = export_dataset(config, source_factory=MockSource)
    replica_poses = read_traj_gt(output/"traj.txt")
    replica_poses[0, 0, 3] += 1.0
    np.savetxt(output/"traj.txt", replica_poses.reshape(-1, 4))
    result = validate_dataset(output, sample_count=3, strict=True)
    assert not result.valid
    assert "traj.txt differs from traj_gt.txt" in result.errors


def test_interactive_export_can_save_before_frame_limit(config):
    interactive = replace(config, trajectory_mode="interactive", frames=10)

    class EarlySaveSource(MockSource):
        stop_reason = "operator_save"

        def frames(self):
            for index, frame in enumerate(super().frames()):
                if index == 2:
                    return
                yield frame

    output = export_dataset(interactive, source_factory=EarlySaveSource)
    metadata = json.loads((output/"metadata.json").read_text())
    assert metadata["frame_count"] == 2
    assert metadata["capture_stop_reason"] == "operator_save"
    assert read_traj_gt(output/"traj_gt.txt").shape == (2, 4, 4)


def test_interactive_abort_returns_valid_partial(config):
    interactive = replace(config, trajectory_mode="interactive", frames=10)

    class AbortSource(MockSource):
        stop_reason = "operator_abort"
        keep_partial = True

        def frames(self):
            for index, frame in enumerate(super().frames()):
                if index == 2:
                    return
                yield frame

    partial = export_dataset(interactive, source_factory=AbortSource)
    assert partial == interactive.output.with_name("output.partial")
    assert partial.is_dir()
    assert not interactive.output.exists()
    assert not (partial/"failure_report.json").exists()
    assert read_traj_gt(partial/"traj_gt.txt").shape == (2, 4, 4)
    result = validate_dataset(partial, sample_count=2, strict=True)
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

