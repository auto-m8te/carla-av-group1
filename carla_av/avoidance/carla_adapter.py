"""CARLA glue for the collision-avoidance module.

Builds a `Perception` from the CARLA world each tick and applies the
`ControlCommand` to the ego vehicle.

Plug-in points (so teammates' modules drop straight in):
    lane_steer_fn(vehicle)        -> float   # Adil's lane module (steering_angle)
    obstacle_distance_fn(vehicle) -> float|None  # Abir's obstacle module
    adjacent_clear_fn(home_wp, target_offset, lane_width) -> bool  # Abir's module
If they are not ready yet, ground-truth fallbacks from the CARLA map/actors are
used so the avoidance logic can still be tested. ALWAYS report in the results
which source was used.
"""

from __future__ import annotations

import math
from typing import Callable, Optional

import carla  # requires CARLA PythonAPI (Python 3.8-3.12)

from avoidance.collision_avoidance import AvoidanceConfig, ControlCommand, Mode, Perception

LaneSteerFn = Callable[[carla.Vehicle], float]
ObstacleFn = Callable[[carla.Vehicle], Optional[float]]
# (home-lane waypoint next to the car, target lateral offset from it in m, lane width) -> free?
AdjacentClearFn = Callable[[carla.Waypoint, float, float], bool]

PATH_MARGIN = 0.3          # m, extra lateral clearance when judging "in path"
DETECTION_RANGE = 60.0     # m
ADJ_LANE_CHECK_BACK = 15.0  # m behind us to check in adjacent lane
ADJ_LANE_CHECK_AHEAD = DETECTION_RANGE  # must cover obstacle + passing room


def speed_of(vehicle: carla.Vehicle) -> float:
    v = vehicle.get_velocity()
    return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def _local_xy(vehicle: carla.Vehicle, location: carla.Location) -> tuple[float, float]:
    """Return (forward, right) coordinates of `location` in the vehicle frame."""
    tf = vehicle.get_transform()
    d = location - tf.location
    fwd, right = tf.get_forward_vector(), tf.get_right_vector()
    return d.x * fwd.x + d.y * fwd.y, d.x * right.x + d.y * right.y


def _travel_waypoint(cmap: carla.Map, vehicle: carla.Vehicle) -> tuple[carla.Waypoint, bool]:
    """Lane waypoint under the car, plus whether the car drives AGAINST that lane's
    direction (true while overtaking through the oncoming lane on Town01/02)."""
    wp = cmap.get_waypoint(vehicle.get_location())
    diff = math.radians(vehicle.get_transform().rotation.yaw - wp.transform.rotation.yaw)
    return wp, math.cos(diff) < 0.0


def _step(wp: carla.Waypoint, dist: float, reverse: bool) -> Optional[carla.Waypoint]:
    nxt = wp.previous(dist) if reverse else wp.next(dist)
    return nxt[0] if nxt else None


def _lateral(wp: carla.Waypoint, loc: carla.Location) -> float:
    d = loc - wp.transform.location
    r = wp.transform.get_right_vector()
    return d.x * r.x + d.y * r.y


def _lane_samples(start: carla.Waypoint, length: float, step_m: float,
                  reverse: bool) -> list[carla.Waypoint]:
    samples, sample, travelled = [], start, 0.0
    while sample is not None and travelled <= length:
        samples.append(sample)
        sample = _step(sample, step_m, reverse)
        travelled += step_m
    return samples


def lane_path(
    vehicle: carla.Vehicle, wp: carla.Waypoint, shift: float, ahead: float,
    behind: float = 0.0, reverse: bool = False, step_m: float = 2.0,
) -> list[tuple[float, float]]:
    """Centreline of a lane as (forward, right) points in the vehicle frame, sorted.

    Follows the lane through `wp` (map geometry only, no other actors), shifted
    sideways by `shift` metres (+ = right of that lane), from `behind` m behind to
    `ahead` m in front along the lane. `reverse` = we drive against the lane direction.
    """
    back = _lane_samples(wp, behind, step_m, not reverse)[1:] if behind > 0 else []
    pts = []
    for sample in back[::-1] + _lane_samples(wp, ahead, step_m, reverse):
        r = sample.transform.get_right_vector()
        loc = sample.transform.location + carla.Location(x=r.x * shift, y=r.y * shift)
        pts.append(_local_xy(vehicle, loc))
    pts.sort()
    return [p for i, p in enumerate(pts) if i == 0 or p[0] > pts[i - 1][0]]


def lane_path_ahead(
    cmap: carla.Map, vehicle: carla.Vehicle, length: float = DETECTION_RANGE, step_m: float = 2.0
) -> list[tuple[float, float]]:
    """Predicted path ahead as (forward, right) points in the vehicle frame.

    Follows the shape of the lane the car is in, shifted sideways so it passes through
    the car. During a lane change the path therefore runs parallel to the lane at the
    car's current offset — the same "in path" definition the ground-truth fallback uses.
    """
    wp, reverse = _travel_waypoint(cmap, vehicle)
    car_lat = _lateral(wp, vehicle.get_location())
    ahead = [p for p in lane_path(vehicle, wp, car_lat, length, reverse=reverse, step_m=step_m)
             if p[0] > 0.0]
    return [(0.0, 0.0)] + ahead


def ground_truth_obstacle_distance(world: carla.World, step_m: float = 1.0) -> ObstacleFn:
    """Fallback obstacle module (stands in for Abir's LiDAR module).

    Walks the car's lane forward (so it works on curves) and reports the gap to the
    nearest vehicle whose body overlaps the car's predicted path.
    """
    cmap = world.get_map()

    def fn(vehicle: carla.Vehicle) -> Optional[float]:
        wp, reverse = _travel_waypoint(cmap, vehicle)
        car_lat = _lateral(wp, vehicle.get_location())
        if reverse:
            car_lat = -car_lat
        ego_half_w = vehicle.bounding_box.extent.y
        ego_half_l = vehicle.bounding_box.extent.x
        others = [a for a in world.get_actors().filter("vehicle.*")
                  if a.id != vehicle.id
                  and a.get_location().distance(vehicle.get_location()) < DETECTION_RANGE + 5]
        if not others:
            return None
        nearest: Optional[float] = None
        for actor in others:
            loc = actor.get_location()
            fwd, _ = _local_xy(vehicle, loc)
            if fwd <= 0:
                continue
            sample, travelled = wp, 0.0
            best = (float("inf"), 0.0)  # (distance to sample, arc length)
            while travelled < DETECTION_RANGE and sample is not None:
                dist = sample.transform.location.distance(loc)
                if dist < best[0]:
                    best = (dist, travelled)
                sample = _step(sample, step_m, reverse)
                travelled += step_m
            arc = best[1]
            anchor = _step(wp, arc, reverse) if arc > 0 else wp
            if anchor is None:
                continue
            lat = _lateral(anchor, loc) * (-1 if reverse else 1)
            overlap = ego_half_w + actor.bounding_box.extent.y + PATH_MARGIN
            if abs(lat - car_lat) < overlap:
                gap = arc - actor.bounding_box.extent.x - ego_half_l
                nearest = gap if nearest is None else min(nearest, gap)
        return nearest

    return fn


def waypoint_lane_steer(world: carla.World, lookahead: float = 6.0) -> LaneSteerFn:
    """Fallback lane module (stands in for Adil's camera module): pure pursuit
    toward the lane centre ahead, in the car's direction of travel."""
    cmap = world.get_map()

    def fn(vehicle: carla.Vehicle) -> float:
        wp, reverse = _travel_waypoint(cmap, vehicle)
        ahead = _step(wp, lookahead, reverse) or wp
        fwd, right = _local_xy(vehicle, ahead.transform.location)
        angle = math.atan2(right, max(fwd, 0.1))
        return max(-1.0, min(1.0, angle / math.radians(35)))

    return fn


class CarlaAvoidanceBridge:
    """Converts CARLA state <-> avoidance module types."""

    def __init__(
        self,
        world: carla.World,
        vehicle: carla.Vehicle,
        config: AvoidanceConfig,
        lane_steer_fn: Optional[LaneSteerFn] = None,
        obstacle_distance_fn: Optional[ObstacleFn] = None,
        adjacent_clear_fn: Optional[AdjacentClearFn] = None,
    ) -> None:
        self.world = world
        self.map = world.get_map()
        self.vehicle = vehicle
        self.config = config
        self.lane_steer_fn = lane_steer_fn or waypoint_lane_steer(world)
        self.obstacle_distance_fn = obstacle_distance_fn or ground_truth_obstacle_distance(world)
        self.adjacent_clear_fn = adjacent_clear_fn  # None -> ground truth
        self._home_wp: Optional[carla.Waypoint] = None
        self.odometer = 0.0
        # Set lazily: before the first world.tick() a new actor reports (0,0,0).
        self._last_location: Optional[carla.Location] = None

    def _adjacent_wp(self, wp: carla.Waypoint) -> Optional[carla.Waypoint]:
        adj = wp.get_left_lane() if self.config.avoid_side < 0 else wp.get_right_lane()
        if adj is None or adj.lane_type != carla.LaneType.Driving:
            return None
        return adj

    def _home_waypoint(self) -> carla.Waypoint:
        """Waypoint on our ORIGINAL lane next to the car, even while in the other lane.

        Tracked geometrically: we walk the home lane forward by however far the car
        moved along it. (Road/lane IDs change between road segments, so matching IDs
        loses the home lane mid-manoeuvre.)
        """
        loc = self.vehicle.get_location()
        if self._home_wp is None:
            self._home_wp = self.map.get_waypoint(loc)
            return self._home_wp
        tf = self._home_wp.transform
        d = loc - tf.location
        fwd = tf.get_forward_vector()
        along = d.x * fwd.x + d.y * fwd.y
        if along > 0.05:
            candidates = self._home_wp.next(along) or [self._home_wp]
            self._home_wp = min(candidates, key=lambda w: w.transform.location.distance(loc))
        return self._home_wp

    def reanchor_home_lane(self) -> None:
        """Call when back in CRUISE so the home lane follows normal lane changes/turns."""
        self._home_wp = self.map.get_waypoint(self.vehicle.get_location())

    def lateral_offset(self) -> float:
        wp = self._home_waypoint()
        d = self.vehicle.get_location() - wp.transform.location
        r = wp.transform.get_right_vector()
        return d.x * r.x + d.y * r.y

    def heading_error(self) -> float:
        """Car yaw minus lane yaw, radians, wrapped to [-pi, pi] (+ = pointing right)."""
        lane_yaw = self._home_waypoint().transform.rotation.yaw
        car_yaw = self.vehicle.get_transform().rotation.yaw
        diff = math.radians(car_yaw - lane_yaw)
        return math.atan2(math.sin(diff), math.cos(diff))

    def adjacent_lane_clear(self) -> bool:
        """Is the avoid-side lane free? The map must have a driving lane there; whether
        it is occupied comes from the plug-in (LiDAR) or the ground-truth fallback."""
        home = self._home_waypoint()
        if self._adjacent_wp(home) is None:
            return False
        if self.adjacent_clear_fn is None:
            return self.ground_truth_adjacent_lane_clear()
        target = self.config.avoid_side * self.config.lane_width
        return self.adjacent_clear_fn(home, target, self.config.lane_width)

    def ground_truth_adjacent_lane_clear(self) -> bool:
        """Fallback (stands in for the LiDAR check): actor positions, measured relative
        to the home lane (road/lane IDs change between road segments, so they can't be
        compared)."""
        home = self._home_waypoint()
        if self._adjacent_wp(home) is None:
            return False
        target = self.config.avoid_side * self.config.lane_width
        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.id == self.vehicle.id:
                continue
            loc = actor.get_location()
            fwd, _ = _local_xy(self.vehicle, loc)
            if not -ADJ_LANE_CHECK_BACK < fwd < ADJ_LANE_CHECK_AHEAD:
                continue
            # Measure the actor against the home lane AT THE ACTOR'S POSITION, so the
            # check stays correct on curves (lane direction there differs from ours).
            anchor = _step(home, abs(fwd), reverse=fwd < 0) if abs(fwd) > 0.5 else home
            actor_offset = _lateral(anchor or home, loc)
            if abs(actor_offset - target) <= self.config.lane_width / 2:
                return False
        return True

    def perceive(self) -> Perception:
        loc = self.vehicle.get_location()
        if self._last_location is not None:
            self.odometer += loc.distance(self._last_location)
        self._last_location = loc
        return Perception(
            lane_steer=self.lane_steer_fn(self.vehicle),
            obstacle_distance=self.obstacle_distance_fn(self.vehicle),
            speed=speed_of(self.vehicle),
            adjacent_lane_clear=self.adjacent_lane_clear(),
            lateral_offset=self.lateral_offset(),
            distance_travelled=self.odometer,
            heading_error=self.heading_error(),
        )

    def apply(self, cmd: ControlCommand) -> None:
        if cmd.mode == Mode.CRUISE and abs(self.lateral_offset()) > self.config.lane_width / 2:
            self.reanchor_home_lane()
        self.vehicle.apply_control(
            carla.VehicleControl(throttle=cmd.throttle, steer=cmd.steer, brake=cmd.brake)
        )
