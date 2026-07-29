from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterator, Optional

import numpy as np

from .config import CaptureConfig
from .coordinate import (
    habitat_sensor_pose_to_opencv_c2w,
    quaternion_xyzw_to_matrix,
)
from .semantic import extract_semantic_metadata
from .sensors import (
    COLOR_UUID, DEPTH_UUID, SEMANTIC_UUID, camera_sensor_spec,
)
from .trajectory import (
    load_replay, turn_toward, wrap_angle, yaw_facing_habitat_direction,
    yaw_from_quaternion_xyzw, yaw_quaternion_xyzw,
)


@dataclass
class CapturedFrame:
    rgb: np.ndarray
    depth_m: np.ndarray
    semantic: Optional[np.ndarray]
    pose_gt: np.ndarray
    trajectory: dict


def import_habitat_sim():
    try:
        import habitat_sim
    except ImportError as exc:
        raise RuntimeError("Habitat-Sim is required for real scene capture") from exc
    return habitat_sim


def quaternion_to_xyzw(rotation: Any) -> np.ndarray:
    if hasattr(rotation, "imag") and hasattr(rotation, "real"):
        return np.array([*np.asarray(rotation.imag), float(rotation.real)])
    if hasattr(rotation, "vector") and hasattr(rotation, "scalar"):
        return np.array([*np.asarray(rotation.vector), float(rotation.scalar)])
    value = np.asarray(rotation, dtype=float).reshape(-1)
    if value.size != 4:
        raise ValueError("cannot convert Habitat quaternion")
    return value


def densify_path(points: list[np.ndarray], step: float) -> list[np.ndarray]:
    output = []
    for start, end in zip(points, points[1:]):
        start, end = np.asarray(start, float), np.asarray(end, float)
        distance = float(np.linalg.norm(end-start))
        if distance <= 1e-8:
            continue
        count = max(1, int(math.ceil(distance/step)))
        for index in range(1, count+1):
            output.append(start + (end-start) * min(index*step/distance, 1.0))
    return output


class HabitatCapture:
    def __init__(self, config: CaptureConfig):
        self.config = config
        self.habitat_sim = import_habitat_sim()
        self.sim = self._create()
        self.agent = self.sim.get_agent(0)
        self.semantic_metadata = extract_semantic_metadata(self.sim.semantic_scene)
        if config.save_semantic and not self.semantic_metadata["instances"]:
            self.sim.close()
            raise RuntimeError("semantic scene contains no instance metadata")
        self.navigable_area = float(self.sim.pathfinder.navigable_area)
        self.collision_count = 0
        self._initialize()

    def _create(self):
        backend = self.habitat_sim.SimulatorConfiguration()
        backend.scene_id = str(self.config.scene)
        backend.scene_dataset_config_file = str(self.config.scene_dataset_config)
        backend.random_seed = self.config.seed
        specs = [
            camera_sensor_spec(self.habitat_sim, COLOR_UUID, self.habitat_sim.SensorType.COLOR, self.config),
            camera_sensor_spec(self.habitat_sim, DEPTH_UUID, self.habitat_sim.SensorType.DEPTH, self.config),
        ]
        if self.config.save_semantic:
            specs.append(camera_sensor_spec(
                self.habitat_sim, SEMANTIC_UUID,
                self.habitat_sim.SensorType.SEMANTIC, self.config,
            ))
        agent = self.habitat_sim.agent.AgentConfiguration()
        agent.sensor_specifications = specs
        simulator = self.habitat_sim.Simulator(
            self.habitat_sim.Configuration(backend, [agent])
        )
        simulator.seed(self.config.seed)
        if not simulator.pathfinder.is_loaded:
            simulator.close()
            raise RuntimeError("NavMesh failed to load")
        if float(simulator.pathfinder.navigable_area) <= 0:
            simulator.close()
            raise RuntimeError("scene has no navigable area")
        return simulator

    def _quat(self, xyzw: np.ndarray):
        from habitat_sim.utils.common import quat_from_coeffs
        return quat_from_coeffs(np.asarray(xyzw, dtype=np.float32))

    def _set_state(self, position: np.ndarray, yaw: float) -> None:
        if not self.sim.pathfinder.is_navigable(position):
            raise ValueError(f"off-navmesh state: {position.tolist()}")
        state = self.agent.get_state()
        state.position = np.asarray(position, dtype=np.float32)
        state.rotation = self._quat(yaw_quaternion_xyzw(yaw))
        self.agent.set_state(state, reset_sensors=True)

    def _initialize(self) -> None:
        if self.config.trajectory_mode == "replay":
            self.replay = load_replay(self.config.trajectory_file)
            return
        start = np.asarray(self.sim.pathfinder.get_random_navigable_point(), float)
        if not np.isfinite(start).all() or not self.sim.pathfinder.is_navigable(start):
            raise RuntimeError("failed to sample navigable start")
        self._set_state(start, 0.0)

    def _sample_path(self) -> list[np.ndarray]:
        start = np.asarray(self.agent.get_state().position, float)
        for _ in range(100):
            path = self.habitat_sim.ShortestPath()
            path.requested_start = start
            path.requested_end = self.sim.pathfinder.get_random_navigable_point()
            if self.sim.pathfinder.find_path(path) and len(path.points) >= 2:
                dense = densify_path(
                    [np.asarray(p, float) for p in path.points],
                    self.config.forward_step,
                )
                if dense:
                    return dense
        raise RuntimeError("failed to sample reachable waypoint")

    def _capture(self, index: int, action: str, collision: bool) -> CapturedFrame:
        observations = self.sim.get_sensor_observations()
        rgb = np.asarray(observations[COLOR_UUID])
        depth = np.asarray(observations[DEPTH_UUID])
        semantic = (
            np.asarray(observations[SEMANTIC_UUID])
            if self.config.save_semantic else None
        )
        shape = (self.config.height, self.config.width)
        if rgb.shape[:2] != shape or depth.shape != shape:
            raise RuntimeError(f"sensor shape mismatch: {rgb.shape}, {depth.shape}")
        if semantic is not None and semantic.shape != shape:
            raise RuntimeError(f"semantic shape mismatch: {semantic.shape}")
        state = self.agent.get_state()
        sensor_state = state.sensor_states[COLOR_UUID]
        sensor_q = quaternion_to_xyzw(sensor_state.rotation)
        pose = habitat_sensor_pose_to_opencv_c2w(
            quaternion_xyzw_to_matrix(sensor_state.position, sensor_q)
        )
        return CapturedFrame(
            rgb, depth, semantic, pose,
            {
                "frame_index": index,
                "agent_position": np.asarray(state.position, float).tolist(),
                "agent_rotation_xyzw": quaternion_to_xyzw(state.rotation).tolist(),
                "sensor_position": np.asarray(sensor_state.position, float).tolist(),
                "sensor_rotation_xyzw": sensor_q.tolist(),
                "action": action,
                "collision": collision,
            },
        )

    def _waypoint_frames(self) -> Iterator[CapturedFrame]:
        targets: list[np.ndarray] = []
        action, collision = "initial", False
        for index in range(self.config.frames):
            yield self._capture(index, action, collision)
            if index + 1 == self.config.frames:
                break
            if not targets:
                targets = self._sample_path()
            current = np.asarray(self.agent.get_state().position, float)
            target = np.asarray(self.sim.pathfinder.snap_point(targets[0]), float)
            if not np.isfinite(target).all() or not self.sim.pathfinder.is_navigable(target):
                targets.clear()
                action, collision = "resample", False
                continue
            direction = target[[0, 2]] - current[[0, 2]]
            if np.linalg.norm(direction) < 1e-5:
                targets.pop(0)
                action, collision = "waypoint_reached", False
                continue
            target_yaw = yaw_facing_habitat_direction(direction)
            current_yaw = yaw_from_quaternion_xyzw(
                quaternion_to_xyzw(self.agent.get_state().rotation)
            )
            error = wrap_angle(target_yaw-current_yaw)
            if abs(math.degrees(error)) > self.config.alignment_tolerance_deg:
                yaw = turn_toward(
                    current_yaw, target_yaw,
                    math.radians(self.config.turn_angle_deg),
                )
                self._set_state(current, yaw)
                action, collision = "turn_in_place", False
                continue
            filtered = np.asarray(self.sim.pathfinder.try_step(current, target), float)
            intended = float(np.linalg.norm(target-current))
            moved = float(np.linalg.norm(filtered-current))
            collision = moved + 1e-5 < intended
            self.collision_count += int(collision)
            self._set_state(filtered, current_yaw)
            if np.linalg.norm(filtered-target) < 0.02:
                targets.pop(0)
            action = "move_forward"

    def _replay_frames(self) -> Iterator[CapturedFrame]:
        if len(self.replay) < self.config.frames:
            raise ValueError("replay has fewer states than requested frames")
        for index, (position, rotation) in enumerate(self.replay[:self.config.frames]):
            if not self.sim.pathfinder.is_navigable(position):
                raise ValueError(f"replay state {index} is off navmesh")
            yaw = yaw_from_quaternion_xyzw(rotation)
            self._set_state(position, yaw)
            yield self._capture(index, "replay", False)

    def frames(self) -> Iterator[CapturedFrame]:
        if self.config.trajectory_mode == "waypoint":
            yield from self._waypoint_frames()
        else:
            yield from self._replay_frames()

    def close(self) -> None:
        self.sim.close()

