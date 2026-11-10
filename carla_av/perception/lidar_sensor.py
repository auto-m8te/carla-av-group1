"""CARLA LiDAR sensor -> obstacle_distance (E3 Obstacle Detection, owner: Abir).

Spawns a roof-mounted ray-cast LiDAR on the ego car, converts each sweep to the
vehicle frame and runs `detect_obstacle`. An instance is callable with the same
signature as the obstacle plug-in point of `CarlaAvoidanceBridge`:

    detector = LidarObstacleDetector(world, ego)
    bridge = CarlaAvoidanceBridge(world, ego, cfg, obstacle_distance_fn=detector)
    ...
    detector.destroy()

Synchronous mode is assumed (one full 360 deg sweep per world.tick()).

"In path" = inside a corridor along the lane ahead (map geometry, i.e. the planned
route; no information about other actors). Without a map path the corridor is bent
by the yaw rate instead, which sweeps onto guardrails on curves and during lane
changes — measured in CARLA Town04, so the map path is the default.
"""

from __future__ import annotations

import logging
import math
import queue
from dataclasses import dataclass, replace
from typing import Optional

import carla
import numpy as np

from avoidance.carla_adapter import lane_path_ahead
from perception.obstacle_detection import CLEAR, Detection, LidarDetectionConfig, detect_obstacle

log = logging.getLogger("lidar")

MIN_SPEED_FOR_CURVATURE = 1.0  # m/s; below this yaw-rate/speed is too noisy
MAX_CURVATURE = 1.0 / 15.0     # 1/m; clamp (tightest urban curve ~15 m radius)


@dataclass(frozen=True)
class LidarMount:
    x: float = 0.0           # m forward of car centre
    z: float = 2.0           # m above ground (roof)
    channels: int = 32
    range: float = 70.0
    points_per_second: int = 600_000
    upper_fov: float = 2.0
    lower_fov: float = -24.0
    noise_stddev: float = 0.0


class LidarObstacleDetector:
    """Callable: detector(vehicle) -> obstacle_distance (m) or None."""

    def __init__(
        self,
        world: carla.World,
        vehicle: carla.Vehicle,
        config: Optional[LidarDetectionConfig] = None,
        mount: LidarMount = LidarMount(),
        use_map_path: bool = True,
    ) -> None:
        self.map = world.get_map() if use_map_path else None
        extent = vehicle.bounding_box.extent
        self.config = replace(
            config or LidarDetectionConfig(),
            ego_half_length=extent.x + vehicle.bounding_box.location.x,
            ego_half_width=extent.y,
        )
        self.mount = mount
        self.vehicle = vehicle
        self.last_detection: Detection = CLEAR
        self._queue: "queue.Queue[carla.LidarMeasurement]" = queue.Queue()
        self.sensor = world.spawn_actor(
            self._blueprint(world, 1.0 / (world.get_settings().fixed_delta_seconds or 0.05)),
            carla.Transform(carla.Location(x=mount.x, z=mount.z)),
            attach_to=vehicle,
        )
        self.sensor.listen(self._queue.put)

    def _blueprint(self, world: carla.World, rotation_hz: float) -> carla.ActorBlueprint:
        bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        m = self.mount
        for key, value in (
            ("channels", m.channels), ("range", m.range), ("points_per_second", m.points_per_second),
            ("rotation_frequency", rotation_hz),  # one full sweep per simulation tick
            ("upper_fov", m.upper_fov), ("lower_fov", m.lower_fov),
            ("noise_stddev", m.noise_stddev),
            ("dropoff_general_rate", 0.0), ("dropoff_intensity_limit", 1.0),
            ("dropoff_zero_intensity", 0.0),
        ):
            bp.set_attribute(key, str(value))
        return bp

    def _latest_points(self) -> Optional[np.ndarray]:
        """Most recent sweep as (N, 3) vehicle-frame points; older sweeps are dropped."""
        measurement = None
        while True:
            try:
                measurement = self._queue.get_nowait()
            except queue.Empty:
                break
        if measurement is None:
            return None
        raw = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4)
        pts = raw[:, :3].astype(float)
        pts[:, 0] += self.mount.x   # sensor frame -> vehicle frame (sensor is not rotated)
        pts[:, 2] += self.mount.z
        return pts

    def _curvature(self, vehicle: carla.Vehicle) -> float:
        """Path curvature from yaw rate / speed (+ = turning right, CARLA convention)."""
        v = vehicle.get_velocity()
        speed = math.hypot(v.x, v.y)
        if speed < MIN_SPEED_FOR_CURVATURE:
            return 0.0
        yaw_rate = math.radians(vehicle.get_angular_velocity().z)
        return max(-MAX_CURVATURE, min(MAX_CURVATURE, yaw_rate / speed))

    def __call__(self, vehicle: carla.Vehicle) -> Optional[float]:
        pts = self._latest_points()
        if pts is None:
            # No new sweep this tick (e.g. first tick): keep the previous answer
            # rather than reporting a possibly false "clear".
            return self.last_detection.distance
        path = None
        if self.map is not None:
            length = self.config.max_range + self.config.ego_half_length
            path = np.asarray(lane_path_ahead(self.map, vehicle, length))
        if path is not None and len(path) >= 2:
            self.last_detection = detect_obstacle(pts, self.config, path=path)
        else:  # no map / end of road: fall back to the yaw-rate arc
            self.last_detection = detect_obstacle(pts, self.config, self._curvature(vehicle))
        return self.last_detection.distance

    def nearest_point_world(self) -> Optional[carla.Location]:
        """World location of the accepted detection (for debug drawing)."""
        p = self.last_detection.nearest_point
        if p is None:
            return None
        return self.vehicle.get_transform().transform(carla.Location(x=p[0], y=p[1], z=p[2]))

    def destroy(self) -> None:
        if self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()
