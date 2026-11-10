"""Queued RGB camera adapter for the lane steering plug-in."""
from __future__ import annotations

import logging
import queue

from perception.sensor_sync import current_frame, wait_for_frame
from pathlib import Path
from typing import Optional

import carla
import cv2
import numpy as np

from perception.lane_detection import LaneDetectionConfig, LaneEstimate, LaneSteering, detect_lane, draw_overlay

log = logging.getLogger(__name__)


class CameraLaneModule:
    def __init__(self, world: carla.World, vehicle: carla.Vehicle,
                 cfg: LaneDetectionConfig = LaneDetectionConfig(),
                 debug_dir: Optional[Path | str] = None, debug_every: int = 20) -> None:
        if debug_every < 1:
            raise ValueError("debug_every must be positive")
        self.cfg = cfg
        self.controller = LaneSteering(cfg)
        self.last_estimate = LaneEstimate(None, None, 0.0, None, None)
        self.debug_dir = Path(debug_dir) if debug_dir is not None else None
        self.debug_every, self._ticks = debug_every, 0
        self._queue: queue.Queue[carla.Image] = queue.Queue()
        self._world = world
        self._destroyed = False
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        for key, value in (("image_size_x", cfg.width), ("image_size_y", cfg.height),
                           ("fov", 90), ("sensor_tick", 0.0)):
            bp.set_attribute(key, str(value))
        self.sensor = world.spawn_actor(bp, carla.Transform(
            carla.Location(x=1.5, z=1.6), carla.Rotation(pitch=-8)), attach_to=vehicle)
        self.sensor.listen(self._queue.put)

    def _refresh(self) -> Optional[np.ndarray]:
        measurement = wait_for_frame(self._queue, current_frame(self._world))
        if measurement is None:
            return None
        return np.frombuffer(measurement.raw_data, dtype=np.uint8).reshape(
            measurement.height, measurement.width, 4)[:, :, :3].copy()

    def __call__(self, vehicle: carla.Vehicle) -> float:
        self._ticks += 1
        bgr = self._refresh()
        if bgr is None:
            return self.controller.steering_angle
        self.last_estimate = detect_lane(bgr, self.cfg, self.controller.prior_width_px)
        steer = self.controller.update(self.last_estimate)
        if self.debug_dir is not None and self._ticks % self.debug_every == 0:
            self.debug_dir.mkdir(parents=True, exist_ok=True)
            path = self.debug_dir / f"lane_{self._ticks:06d}.png"
            if not cv2.imwrite(str(path), draw_overlay(bgr, self.last_estimate, self.cfg)):
                log.warning("Could not save lane overlay: %s", path)
        return steer

    def destroy(self) -> None:
        if self._destroyed:
            return
        self._destroyed = True
        try:
            if self.sensor.is_alive:
                self.sensor.stop()
        except RuntimeError as exc:
            log.warning("Could not stop camera: %s", exc)
        try:
            if self.sensor.is_alive:
                self.sensor.destroy()
        except RuntimeError as exc:
            log.warning("Could not destroy camera: %s", exc)
