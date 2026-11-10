"""Run lane, obstacle and sharp-curve test scenarios in CARLA across several towns and log one CSV per run.

Scenarios (Trello card "Obstacle + avoidance: test scenarios + docs"):
    empty_lane     - no obstacle, just lane following
    obstacle_stop  - parked car ahead AND adjacent lane blocked -> must stop
    obstacle_avoid - parked car ahead, adjacent lane free       -> must steer around
    curve_lane     - sharp curve, no obstacle                  -> follow lane
    curve_stop     - parked car at curve apex, adjacent blocked -> must stop

Every run is randomised (seeded, so reproducible): start location, obstacle
distance and cruise speed. On towns without a same-direction second lane
(Town01/02) the car overtakes through the oncoming lane, which is empty.

Usage (CARLA server running on localhost:2000):
    python -m scenarios.run_scenarios --towns Town01 Town03 Town04 --runs 5
Output: results/<town>/<scenario>_run<N>.csv  (consumed by analysis/analyze_results.py)
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import random
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Optional

import carla

from avoidance.carla_adapter import (
    CarlaAvoidanceBridge, ObstacleFn, ground_truth_obstacle_distance, speed_of, waypoint_lane_steer,
)
from avoidance.collision_avoidance import AvoidanceConfig, CollisionAvoidance, Mode
from perception.camera_sensor import CameraLaneModule
from perception.lidar_sensor import LidarMount, LidarObstacleDetector

log = logging.getLogger("scenarios")

SCENARIOS = ("empty_lane", "obstacle_stop", "obstacle_avoid")
CURVE_SCENARIOS = ("curve_lane", "curve_stop")
MOVING_SCENARIOS = ("lead_vehicle", "lead_brake", "oncoming_overtake")
CURVE_WINDOW_M = 80.0
CURVE_HEADING_RANGE = (45.0, 120.0)


DEFAULT_TOWNS = ("Town01", "Town02", "Town03", "Town04", "Town05", "Town10HD")
FIXED_DT = 0.05
MAX_SECONDS = 50.0
STOP_HOLD_SECONDS = 3.0
PASS_DONE_M = 25.0
EMPTY_LANE_M = 100.0
OBSTACLE_RANGE_M = (35.0, 55.0)
TARGET_SPEED_RANGE = (6.0, 10.0)   # m/s, ~22-36 km/h urban speeds
ROAD_AFTER_OBSTACLE_M = 45.0       # room needed to pass and merge back
PROBE_STEP_M = 5.0
MAX_HEADING_CHANGE_DEG = 25.0      # mild curves allowed, no sharp turns
CSV_FIELDS = (
    "t", "x", "y", "speed", "mode", "obstacle_distance", "lateral_offset",
    "adjacent_lane_clear", "throttle", "steer", "brake", "odometer", "collision",
    "gt_obstacle_distance",     # ground truth, logged alongside to measure perception error
    "gt_adjacent_lane_clear",
    "det_tag", "adj_tag",
    "obstacle_speed", "other_speed", "other_gap",
    "lane_conf", "lane_offset_cam", "gt_lane_steer",
)
PERCEPTION_SOURCES = ("groundtruth", "lidar")


class NoUsableCurve(RuntimeError):
    """The town has no available stretch for a curve run."""


class NoOncomingRoad(RuntimeError):
    """No available road has an adjacent lane in the opposite direction."""



@dataclass(frozen=True)
class RunParams:
    town: str
    scenario: str
    run: int
    obstacle_m: float
    target_speed: float
    realtime: bool = False   # pace ticks to wall-clock time (for demo videos)
    hud: bool = False        # draw controller mode / speed / gap above the car
    obstacle_source: str = "groundtruth"  # "lidar" = Abir's LiDAR module (E3)
    adjacent_source: str = "groundtruth"  # "lidar" = adjacent-lane check from LiDAR too
    lane_source: str = "groundtruth"
    lane_debug: Optional[str] = None
    lidar_debug: bool = False
    lidar_noise: float = 0.0   # LiDAR range noise std-dev in metres (robustness test)


def scripted_speed_control(speed: float, target: float) -> tuple[float, float]:
    """Small feed-forward/P controller returning throttle and brake."""
    if target <= 0:
        return 0.0, 1.0
    error = target - speed
    if error < -0.2:
        return 0.0, min(1.0, -0.4 * error)
    return max(0.0, min(0.75, 0.25 + 0.3 * error)), 0.0


def scripted_target_speed(scenario: str, target: float, travelled: float) -> float:
    return 0.0 if scenario == "lead_brake" and travelled >= 40.0 else target


def lidar_mount(params: "RunParams", cfg: AvoidanceConfig) -> LidarMount:
    """Standard 32-channel / 70 m LiDAR; long-range 64-channel unit for moving traffic.

    An overtake against oncoming traffic needs ~150 m of visibly free lane. At 60-70 m a
    32-channel scan puts too few points on a car (detections flicker), so the moving-traffic
    scenarios use a 64-channel, 185 m sensor (typical of long-range automotive LiDAR).
    """
    if params.scenario in MOVING_SCENARIOS:
        return LidarMount(semantic=params.lidar_debug, range=max(70.0, cfg.adjacent_detection_range + 5.0),
                          channels=64, points_per_second=1_300_000, noise_stddev=params.lidar_noise)
    return LidarMount(semantic=params.lidar_debug, noise_stddev=params.lidar_noise)


def scenario_config(base: AvoidanceConfig, params: RunParams, lane_width: float) -> AvoidanceConfig:
    cfg = replace(base, target_speed=params.target_speed, lane_width=lane_width)
    if params.scenario in MOVING_SCENARIOS:
        # Hold outside the emergency zone so a waiting ego can resume its overtake.
        cfg = replace(cfg, adjacent_detection_range=180.0,
                      safety_margin=max(cfg.safety_margin, cfg.standstill_gap))
    return cfg


class ScriptedDriver:
    """One moving actor, controlled once per simulation tick."""

    def __init__(self, world: carla.World, actor: carla.Vehicle,
                 scenario: str, target_speed: float) -> None:
        self.actor, self.scenario, self.target_speed = actor, scenario, target_speed
        self.steer = waypoint_lane_steer(world)
        self.gap = ground_truth_obstacle_distance(world)
        self.travelled = 0.0
        self._last_location: Optional[carla.Location] = None

    def step(self) -> None:
        loc = self.actor.get_location()
        if self._last_location is not None:
            self.travelled += loc.distance(self._last_location)
        self._last_location = loc
        target = scripted_target_speed(self.scenario, self.target_speed, self.travelled)
        throttle, brake = scripted_speed_control(speed_of(self.actor), target)
        self.actor.apply_control(carla.VehicleControl(
            throttle=throttle, brake=brake, steer=self.steer(self.actor)))

    def log_values(self, ego: carla.Vehicle) -> dict:
        half_lengths = ego.bounding_box.extent.x + self.actor.bounding_box.extent.x
        distance = ego.get_location().distance(self.actor.get_location())
        gap = distance - half_lengths
        if self.scenario != "oncoming_overtake":
            gap = self.gap(ego)
        return {"other_speed": speed_of(self.actor), "other_gap": _blank(gap)}


def _adjacent(wp: carla.Waypoint, side: int) -> Optional[carla.Waypoint]:
    adj = wp.get_left_lane() if side < 0 else wp.get_right_lane()
    if adj is None or adj.lane_type != carla.LaneType.Driving:
        return None
    return adj


def _yaw_delta(a: float, b: float) -> float:
    d = math.radians(a - b)
    return abs(math.degrees(math.atan2(math.sin(d), math.cos(d))))


def road_is_usable(start: carla.Waypoint, length: float, side: int) -> bool:
    """Whole stretch: no junction, adjacent driving lane present, no sharp curve."""
    probe, travelled = start, 0.0
    while travelled < length:
        if probe.is_junction or _adjacent(probe, side) is None:
            return False
        if _yaw_delta(probe.transform.rotation.yaw, start.transform.rotation.yaw) > MAX_HEADING_CHANGE_DEG:
            return False
        nxt = probe.next(PROBE_STEP_M)
        if not nxt:
            return False
        probe, travelled = nxt[0], travelled + PROBE_STEP_M
    return True


def _curve_profile(start: carla.Waypoint, length: float) -> list[tuple[float, float]]:
    """Arc length and cumulative absolute heading change at each probe."""
    profile = [(0.0, 0.0)]
    probe, travelled, total = start, 0.0, 0.0
    while travelled < length:
        step = min(PROBE_STEP_M, length - travelled)
        nxt = probe.next(step)
        if not nxt:
            return []
        total += _yaw_delta(nxt[0].transform.rotation.yaw, probe.transform.rotation.yaw)
        probe, travelled = nxt[0], travelled + step
        profile.append((travelled, total))
    return profile


def curve_is_usable(start: carla.Waypoint, length: float, side: int) -> bool:
    """Smooth sharp curve in the first 80 m, with a usable whole stretch."""
    probe, travelled, total = start, 0.0, 0.0
    length = max(length, CURVE_WINDOW_M)
    while True:
        if probe.is_junction or _adjacent(probe, side) is None:
            return False
        if travelled >= length:
            break
        step = min(PROBE_STEP_M, length - travelled)
        nxt = probe.next(step)
        if not nxt:
            return False
        delta = _yaw_delta(nxt[0].transform.rotation.yaw, probe.transform.rotation.yaw)
        if delta > MAX_HEADING_CHANGE_DEG:
            return False
        if travelled < CURVE_WINDOW_M:
            total += delta
        probe, travelled = nxt[0], travelled + step
    return CURVE_HEADING_RANGE[0] <= total <= CURVE_HEADING_RANGE[1]


def curve_obstacle_distance(start: carla.Waypoint) -> float:
    """Interpolate the half-heading apex, clamped to the obstacle range."""
    profile = _curve_profile(start, CURVE_WINDOW_M)
    half = profile[-1][1] / 2
    for (before_m, before), (after_m, after) in zip(profile, profile[1:]):
        if after >= half and after > before:
            apex = before_m + (after_m - before_m) * (half - before) / (after - before)
            return max(25.0, min(55.0, apex))
    raise ValueError("Curve has no heading change")


def candidate_starts(cmap: carla.Map, length: float, side: int, rng: random.Random,
                     predicate: Callable[[carla.Waypoint, float, int], bool] = road_is_usable,
                     ) -> list[carla.Waypoint]:
    wps = [w for w in cmap.generate_waypoints(10.0) if w.lane_type == carla.LaneType.Driving]
    rng.shuffle(wps)
    return [w for w in wps if predicate(w, length, side)]


def spawn_at(world: carla.World, bp_name: str, wp: carla.Waypoint) -> Optional[carla.Actor]:
    bp = world.get_blueprint_library().find(bp_name)
    tf = carla.Transform(wp.transform.location + carla.Location(z=0.5), wp.transform.rotation)
    return world.try_spawn_actor(bp, tf)


def _scenario_vehicles(start: carla.Waypoint, params: RunParams,
                       cfg: AvoidanceConfig, rng: random.Random) -> list[tuple[str, carla.Waypoint, bool]]:
    if params.scenario in ("empty_lane", "curve_lane"):
        return []
    distance = curve_obstacle_distance(start) if params.scenario == "curve_stop" else params.obstacle_m
    obstacle = start.next(distance)[0]
    moving_lead = params.scenario in ("lead_vehicle", "lead_brake")
    vehicles = [("vehicle.audi.a2", obstacle, moving_lead)]
    if params.scenario in ("obstacle_stop", "curve_stop", "lead_brake"):
        anchor = start.next(params.obstacle_m + 20.0)[0] if moving_lead else obstacle
        vehicles.append(("vehicle.nissan.patrol", _adjacent(anchor, cfg.avoid_side), False))
    if params.scenario == "oncoming_overtake":
        anchor = start.next(distance + rng.uniform(40.0, 60.0))[0]
        adjacent = _adjacent(anchor, cfg.avoid_side)
        if adjacent is None or adjacent.lane_id * anchor.lane_id >= 0:
            return []
        vehicles.append(("vehicle.nissan.micra", adjacent, True))
    return vehicles


def _spawn_scenario_actors(world: carla.World, start: carla.Waypoint, params: RunParams,
                           cfg: AvoidanceConfig, rng: random.Random) -> list[carla.Actor]:
    vehicles = _scenario_vehicles(start, params, cfg, rng)
    if params.scenario == "oncoming_overtake" and len(vehicles) != 2:
        return []
    ego = spawn_at(world, "vehicle.tesla.model3", start)
    if ego is None:
        return []
    actors = [ego]
    for name, wp, moving in vehicles:
        actor = spawn_at(world, name, wp) if wp is not None else None
        if actor is None:
            for spawned in actors:
                spawned.destroy()
            return []
        actor.set_simulate_physics(moving)
        actors.append(actor)
    return actors


def setup_scenario(world: carla.World, params: RunParams, cfg: AvoidanceConfig, rng: random.Random):
    """Spawn ego (+ obstacle, + blocker). Returns (actors, start_waypoint)."""
    cmap = world.get_map()
    length = params.obstacle_m + ROAD_AFTER_OBSTACLE_M
    is_curve = params.scenario in CURVE_SCENARIOS
    if is_curve:
        length = max(EMPTY_LANE_M, CURVE_WINDOW_M)
    if params.scenario in MOVING_SCENARIOS:
        length = max(140.0 if params.scenario != "oncoming_overtake" else params.obstacle_m + 60.0, length)
    predicate = curve_is_usable if is_curve else road_is_usable
    for start in candidate_starts(cmap, length, cfg.avoid_side, rng, predicate)[:40]:
        if params.scenario == "oncoming_overtake" and _adjacent(start, cfg.avoid_side).lane_id * start.lane_id >= 0:
            continue
        actors = _spawn_scenario_actors(world, start, params, cfg, rng)
        if actors:
            return actors, start
    if is_curve:
        raise NoUsableCurve(f"{params.town}: no usable curve for {params.scenario}")
    if params.scenario == "oncoming_overtake":
        raise NoOncomingRoad(f"{params.town}: no usable oncoming lane for {params.scenario}")
    raise RuntimeError(f"{params.town}: no usable road for {params.scenario}")


def is_finished(scenario: str, row: dict, stopped_for: float, passed_m: float) -> bool:
    if row["collision"]:
        return True
    if scenario in ("obstacle_stop", "curve_stop", "lead_brake"):
        return stopped_for >= STOP_HOLD_SECONDS
    if scenario in ("obstacle_avoid", "oncoming_overtake"):
        return passed_m >= PASS_DONE_M and row["mode"] == Mode.CRUISE.value
    return row["odometer"] >= (120.0 if scenario == "lead_vehicle" else EMPTY_LANE_M)


def follow_camera(world: carla.World, ego: carla.Actor) -> None:
    tf = ego.get_transform()
    world.get_spectator().set_transform(carla.Transform(
        tf.location - tf.get_forward_vector() * 10 + carla.Location(z=6),
        carla.Rotation(pitch=-20, yaw=tf.rotation.yaw),
    ))


HUD_COLORS = {
    Mode.CRUISE: (0, 200, 0),
    Mode.FOLLOW: (220, 200, 0),
    Mode.BRAKE: (255, 140, 0),
    Mode.STOPPED: (220, 0, 0),
    Mode.EMERGENCY_STOP: (255, 0, 255),
    Mode.ABORT: (255, 170, 0),
    Mode.AVOID: (0, 120, 255),
    Mode.PASSING: (0, 120, 255),
    Mode.RETURN: (0, 220, 220),
}


def draw_hud(world: carla.World, ego: carla.Actor, mode: Mode, speed: float,
             obstacle_distance: Optional[float]) -> None:
    gap = "clear" if obstacle_distance is None else f"obstacle {obstacle_distance:.0f} m"
    text = f"{mode.value}  {speed * 3.6:.0f} km/h  {gap}"
    world.debug.draw_string(
        ego.get_location() + carla.Location(z=2.5), text, draw_shadow=True,
        color=carla.Color(*HUD_COLORS[mode]), life_time=FIXED_DT * 1.5,
    )


def _blank(value: Optional[float]) -> object:
    return "" if value is None else value


def draw_detection(world: carla.World, detector: Optional[LidarObstacleDetector]) -> None:
    """Red dot on the LiDAR point the obstacle distance was measured to."""
    point = detector.nearest_point_world() if detector is not None else None
    if point is not None:
        world.debug.draw_point(point, size=0.15, color=carla.Color(255, 0, 0), life_time=FIXED_DT * 1.5)


def lane_log_values(camera: Optional[CameraLaneModule], gt_steer: float) -> dict:
    estimate = camera.last_estimate if camera is not None else None
    return {"lane_conf": estimate.confidence if estimate is not None else "",
            "lane_offset_cam": _blank(estimate.lateral_offset_m) if estimate is not None else "",
            "gt_lane_steer": gt_steer}


def drive_and_log(world, ego, bridge, ctrl, params, out: Path, collided: dict,
                  gt_fn: ObstacleFn, detector: Optional[LidarObstacleDetector] = None,
                  other: Optional[ScriptedDriver] = None,
                  camera: Optional[CameraLaneModule] = None) -> None:
    gt_lane_fn = waypoint_lane_steer(world)
    partial = out.with_suffix(".partial")  # renamed only when the run ends normally
    with partial.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        writer.writeheader()
        t, stopped_for = 0.0, 0.0
        while t < MAX_SECONDS:
            tick_start = time.perf_counter()
            if other is not None:
                other.step()
            p = bridge.perceive()
            cmd = ctrl.step(p)
            bridge.apply(cmd)
            loc = ego.get_location()
            row = dict(
                t=round(t, 3), x=loc.x, y=loc.y, speed=p.speed, mode=cmd.mode.value,
                obstacle_distance=_blank(p.obstacle_distance),
                lateral_offset=p.lateral_offset, adjacent_lane_clear=int(p.adjacent_lane_clear),
                throttle=cmd.throttle, steer=cmd.steer, brake=cmd.brake,
                odometer=p.distance_travelled, collision=int(collided["hit"]),
                gt_obstacle_distance=_blank(
                    gt_fn(ego) if params.obstacle_source == "lidar" else p.obstacle_distance
                ),
                gt_adjacent_lane_clear=int(
                    bridge.ground_truth_adjacent_lane_clear()
                ),
            )
            row["det_tag"], row["adj_tag"] = (
                detector.debug_tags() if detector is not None and params.lidar_debug else ("", "")
            )
            row["obstacle_speed"] = _blank(p.obstacle_speed)
            row.update(other.log_values(ego) if other is not None else
                       {"other_speed": "", "other_gap": ""})
            row.update(lane_log_values(camera, gt_lane_fn(ego)))
            writer.writerow(row)
            stopped_for = stopped_for + FIXED_DT if cmd.mode == Mode.STOPPED else 0.0
            if is_finished(params.scenario, row, stopped_for, p.distance_travelled - params.obstacle_m):
                break
            follow_camera(world, ego)
            if params.hud:
                draw_hud(world, ego, cmd.mode, p.speed, p.obstacle_distance)
                draw_detection(world, detector)
            world.tick()
            t += FIXED_DT
            if params.realtime:
                time.sleep(max(0.0, FIXED_DT - (time.perf_counter() - tick_start)))
    partial.replace(out)


def _teardown(client: carla.Client, world: carla.World, settings: carla.WorldSettings,
              actors: list[carla.Actor]) -> None:
    """Flush stopped sensors and batch destruction before leaving synchronous mode."""
    sensors: list[carla.Actor] = []
    others: list[carla.Actor] = []
    for actor in actors:
        try:
            if actor.type_id.startswith("sensor."):
                sensors.append(actor)
                if actor.is_alive:
                    actor.stop()
            else:
                others.append(actor)
        except Exception as exc:
            log.warning("Could not stop or classify actor during teardown: %s", exc)

    for _ in range(2):
        _teardown_tick(world)
    try:
        responses = client.apply_batch_sync(
            [carla.command.DestroyActor(actor) for actor in sensors + others], True,
        )
        for response in responses:
            if response.error:
                log.warning("Could not destroy actor during teardown: %s", response.error)
    except Exception as exc:
        log.warning("Could not batch destroy actors during teardown: %s", exc)
    _teardown_tick(world)
    try:
        settings.synchronous_mode, settings.fixed_delta_seconds = False, None
        world.apply_settings(settings)
    except Exception as exc:
        log.warning("Could not restore asynchronous mode during teardown: %s", exc)


def _teardown_tick(world: carla.World) -> None:
    """Allow cleanup to continue even if the server no longer accepts ticks."""
    try:
        world.tick()
    except RuntimeError as exc:
        log.warning("Could not tick world during teardown: %s", exc)
    except Exception as exc:
        log.warning("Unexpected error ticking world during teardown: %s", exc)


def _drive_actors(world: carla.World, actors: list[carla.Actor], cfg: AvoidanceConfig,
                  params: RunParams, rng: random.Random, out: Path) -> bool:
    ego = actors[0]
    collided = {"hit": False}
    sensor = world.spawn_actor(
        world.get_blueprint_library().find("sensor.other.collision"), carla.Transform(), attach_to=ego
    )
    sensor.listen(lambda _e: collided.__setitem__("hit", True))
    actors.append(sensor)
    detector = None
    if "lidar" in (params.obstacle_source, params.adjacent_source):
        detector = LidarObstacleDetector(world, ego, mount=lidar_mount(params, cfg),
                                         avoidance_config=cfg)
        actors.append(detector.sensor)
    camera = None
    if params.lane_source == "camera":
        debug_dir = (Path(params.lane_debug) / params.town / f"{params.scenario}_run{params.run}"
                     if params.lane_debug else None)
        camera = CameraLaneModule(world, ego, debug_dir=debug_dir)
        actors.append(camera.sensor)
    world.tick()
    gt_fn = ground_truth_obstacle_distance(world)
    bridge = CarlaAvoidanceBridge(
        world, ego, cfg, lane_steer_fn=camera,
        obstacle_distance_fn=detector if params.obstacle_source == "lidar" else gt_fn,
        adjacent_clear_fn=detector.adjacent_lane_clear if params.adjacent_source == "lidar" else None,
        tracking_distance_fn=(ground_truth_obstacle_distance(world, interpolate=True)
                              if params.obstacle_source == "groundtruth" else None),
    )
    other = None
    if params.scenario in MOVING_SCENARIOS:
        oncoming = params.scenario == "oncoming_overtake"
        actor = actors[2] if oncoming else actors[1]
        target = rng.uniform(6.0, 8.0) if oncoming else params.target_speed * rng.uniform(0.4, 0.6)
        other = ScriptedDriver(world, actor, params.scenario, target)
    drive_and_log(world, ego, bridge, CollisionAvoidance(cfg), params, out, collided, gt_fn, detector, other, camera)
    return collided["hit"]


def run_once(client: carla.Client, params: RunParams, out_dir: Path, base_cfg: AvoidanceConfig) -> Optional[Path]:
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode, settings.fixed_delta_seconds = True, FIXED_DT
    world.apply_settings(settings)
    rng = random.Random(f"{params.town}-{params.scenario}-{params.run}")

    actors: list = []
    try:
        try:
            actors, start = setup_scenario(world, params, base_cfg, rng)
        except (NoUsableCurve, NoOncomingRoad) as exc:
            log.warning("%s; skipping run %d", exc, params.run)
            return None
        if params.scenario == "curve_stop":
            params = replace(params, obstacle_m=curve_obstacle_distance(start))
        adj = _adjacent(start, base_cfg.avoid_side)
        lane_gap = start.transform.location.distance(adj.transform.location)
        cfg = scenario_config(base_cfg, params, lane_gap)
        out = out_dir / f"{params.scenario}_run{params.run}.csv"
        collision = _drive_actors(world, actors, cfg, params, rng, out)
        oncoming = adj.lane_id * start.lane_id < 0
        log.info("%s %s run %d: obstacle %.0f m, %.1f m/s, lane %.2f m%s -> collision=%s",
                 params.town, params.scenario, params.run, params.obstacle_m, params.target_speed,
                 lane_gap, " (overtake via oncoming lane)" if oncoming else "", collision)
        return out
    finally:
        _teardown(client, world, settings, actors)


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="seconds to wait for CARLA (map loads on a cold start can exceed 60 s)")
    ap.add_argument("--towns", nargs="+", default=list(DEFAULT_TOWNS))
    ap.add_argument("--scenario", choices=(*SCENARIOS, *CURVE_SCENARIOS, *MOVING_SCENARIOS,
                                         "all", "curves", "moving", "everything"), default="all")
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--first-run", type=int, default=1, help="start numbering at this run (re-run one seed)")
    ap.add_argument("--out", default="results")
    ap.add_argument("--resume", action="store_true", help="skip runs whose CSV already exists")
    ap.add_argument("--realtime", action="store_true", help="run at real speed (for recording a demo video)")
    ap.add_argument("--hud", action="store_true", help="show controller mode, speed and obstacle gap above the car")
    ap.add_argument("--obstacle", choices=PERCEPTION_SOURCES, default="groundtruth",
                    help="obstacle_distance source: CARLA ground truth or Abir's LiDAR module")
    ap.add_argument("--adjacent", choices=PERCEPTION_SOURCES, default="groundtruth",
                    help="adjacent-lane-free check: CARLA ground truth or LiDAR")
    ap.add_argument("--lidar-noise", type=float, default=0.0,
                    help="LiDAR range noise std-dev in metres (semantic LiDAR ignores it)")
    ap.add_argument("--lidar-debug", action="store_true",
                    help="use semantic LiDAR and log the object tag behind detections")
    ap.add_argument("--lane", choices=("groundtruth", "camera"), default="groundtruth")
    ap.add_argument("--lane-debug", metavar="DIR", help="save camera lane overlays")
    return ap.parse_args()


def main() -> None:
    args = _parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    base_cfg = AvoidanceConfig()
    groups = {"all": SCENARIOS, "curves": CURVE_SCENARIOS, "moving": MOVING_SCENARIOS,
              "everything": SCENARIOS + CURVE_SCENARIOS + MOVING_SCENARIOS}
    scenarios = groups.get(args.scenario, (args.scenario,))
    for town in args.towns:
        # Reloading the map that is already loaded has crashed CARLA 0.9.16 on Windows.
        if not client.get_world().get_map().name.endswith(town):
            client.load_world(town)
        out_dir = Path(args.out) / town
        out_dir.mkdir(parents=True, exist_ok=True)
        for scenario in scenarios:
            for i in range(args.first_run, args.runs + 1):
                rng = random.Random(f"params-{town}-{scenario}-{i}")
                params = RunParams(
                    town=town, scenario=scenario, run=i,
                    obstacle_m=round(rng.uniform(*( (30.0, 40.0) if scenario in
                        ("lead_vehicle", "lead_brake") else OBSTACLE_RANGE_M)), 1),
                    target_speed=round(rng.uniform(*TARGET_SPEED_RANGE), 2),
                    lane_source=args.lane, lane_debug=args.lane_debug,
                    lidar_debug=args.lidar_debug, lidar_noise=args.lidar_noise, realtime=args.realtime, hud=args.hud, obstacle_source=args.obstacle, adjacent_source=args.adjacent,
                )
                if args.resume and (out_dir / f"{scenario}_run{i}.csv").exists():
                    continue
                run_once(client, params, out_dir, base_cfg)


if __name__ == "__main__":
    main()
