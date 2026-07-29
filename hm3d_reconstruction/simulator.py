from __future__ import annotations

import math
import time
from collections import deque
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


class InteractiveCaptureAborted(RuntimeError):
    """Signal operator cancellation while retaining transactional output."""


def interactive_displacement(yaw: float, distance: float) -> np.ndarray:
    """Return a Habitat-world displacement along the current heading."""
    return np.array(
        [-math.sin(yaw) * distance, 0.0, -math.cos(yaw) * distance],
        dtype=np.float64,
    )


class TkInteractiveWindow:
    """Live RGB viewer with a non-blocking keyboard command queue."""

    KEY_COMMANDS = {
        "w": "forward",
        "s": "backward",
        "a": "left",
        "d": "right",
        "r": "record",
        "p": "pause",
        "q": "save",
        "escape": "abort",
    }

    def __init__(self, width: int, height: int, scale: float = 1.5):
        try:
            import tkinter as tk
            from PIL import ImageTk
        except ImportError as exc:
            raise RuntimeError(
                "interactive mode requires Tk and Pillow ImageTk"
            ) from exc
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            raise RuntimeError(
                "interactive mode requires a graphical display (DISPLAY)"
            ) from exc
        self._tk = tk
        self._image_tk = ImageTk
        self._commands = deque()
        self._closed = False
        self.root.title("HM3D interactive capture")
        self.display_size = (
            max(1, int(round(width * scale))),
            max(1, int(round(height * scale))),
        )
        self.canvas = tk.Label(self.root)
        self.canvas.pack()
        self.status = tk.Label(self.root, anchor="w", padx=8, pady=5)
        self.status.pack(fill="x")
        self.root.bind("<KeyPress>", self._on_key)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.focus_force()
        self._photo = None

    def _on_key(self, event) -> None:
        command = self.KEY_COMMANDS.get(str(event.keysym).casefold())
        if command is not None:
            self._commands.append(command)

    def _on_close(self) -> None:
        if not self._closed:
            self._commands.append("abort")
            self._closed = True

    def wait_command(
        self, rgb: np.ndarray, recording: bool, frames: int, maximum: int
    ) -> str:
        from PIL import Image

        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] < 3:
            raise ValueError("interactive RGB preview must be HxWx3 or HxWx4")
        preview = Image.fromarray(image[:, :, :3])
        if preview.size != self.display_size:
            resampling = getattr(Image, "Resampling", Image)
            preview = preview.resize(self.display_size, resampling.BILINEAR)
        self._photo = self._image_tk.PhotoImage(preview)
        self.canvas.configure(image=self._photo)
        state = "RECORDING" if recording else "PAUSED"
        self.status.configure(text=f"{state}    frames {frames}/{maximum}")
        while not self._commands:
            if self._closed:
                return "abort"
            try:
                self.root.update_idletasks()
                self.root.update()
            except self._tk.TclError:
                return "abort"
            time.sleep(0.01)
        return self._commands.popleft()

    def close(self) -> None:
        self._closed = True
        try:
            self.root.destroy()
        except self._tk.TclError:
            pass


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
        self._interactive_window = None
        self.stop_reason = "frame_limit"
        self.keep_partial = False
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

    def _interactive_move(self, forward: bool) -> tuple[str, bool]:
        state = self.agent.get_state()
        current = np.asarray(state.position, dtype=np.float64)
        yaw = yaw_from_quaternion_xyzw(quaternion_to_xyzw(state.rotation))
        distance = self.config.forward_step * (1.0 if forward else -1.0)
        target = current + interactive_displacement(yaw, distance)
        filtered = np.asarray(self.sim.pathfinder.try_step(current, target), float)
        moved = float(np.linalg.norm(filtered-current))
        collision = moved + 1e-5 < abs(distance)
        self.collision_count += int(collision)
        self._set_state(filtered, yaw)
        return ("move_forward" if forward else "move_backward"), collision

    def _interactive_turn(self, left: bool) -> tuple[str, bool]:
        state = self.agent.get_state()
        position = np.asarray(state.position, dtype=np.float64)
        yaw = yaw_from_quaternion_xyzw(quaternion_to_xyzw(state.rotation))
        delta = math.radians(self.config.turn_angle_deg)
        self._set_state(position, yaw + (delta if left else -delta))
        return ("turn_left" if left else "turn_right"), False

    def _interactive_frames(self) -> Iterator[CapturedFrame]:
        window = TkInteractiveWindow(
            self.config.width, self.config.height, self.config.display_scale
        )
        self._interactive_window = window
        recording = False
        captured = 0
        preview = self._capture(0, "preview", False)
        while captured < self.config.frames:
            command = window.wait_command(
                preview.rgb, recording, captured, self.config.frames
            )
            if command == "abort":
                self.stop_reason = "operator_abort"
                if captured == 0:
                    raise InteractiveCaptureAborted(
                        "interactive capture aborted before recording started"
                    )
                self.keep_partial = True
                return
            if command == "save":
                if captured == 0:
                    continue
                self.stop_reason = "operator_save"
                return
            if command == "record":
                recording = True
                if captured == 0:
                    preview = self._capture(captured, "record_start", False)
                    captured += 1
                    yield preview
                continue
            if command == "pause":
                recording = False
                continue
            action, collision = "preview", False
            if command == "forward":
                action, collision = self._interactive_move(True)
            elif command == "backward":
                action, collision = self._interactive_move(False)
            elif command == "left":
                action, collision = self._interactive_turn(True)
            elif command == "right":
                action, collision = self._interactive_turn(False)
            preview = self._capture(captured, action, collision)
            if recording:
                captured += 1
                yield preview
        self.stop_reason = "frame_limit"

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
        elif self.config.trajectory_mode == "interactive":
            yield from self._interactive_frames()
        else:
            yield from self._replay_frames()

    def close(self) -> None:
        if self._interactive_window is not None:
            self._interactive_window.close()
        self.sim.close()

