"""CARLA glue for the collision-avoidance module.

Builds a `Perception` from the CARLA world each tick and applies the
`ControlCommand` to the ego vehicle.

Plug-in points (so teammates' modules drop straight in):
    lane_steer_fn(vehicle)        -> float   # Adil's lane module (steering_angle)
    obstacle_distance_fn(vehicle) -> float|None  # Abir's obstacle module
    adjacent_clear_fn(home_wp, target_offset, lane_width, behind) -> bool
If they are not ready yet, ground-truth fallbacks from the CARLA map/actors are
used so the avoidance logic can still be tested. ALWAYS report in the results
which source was used.
"""

from __future__ import annotations

import math
import logging
import inspect
from typing import Callable, Optional

import carla  # requires CARLA PythonAPI (Python 3.8-3.12)

from avoidance.collision_avoidance import AvoidanceConfig, ControlCommand, Mode, Perception
from navigation.route_follower import RouteFollower
from perception.tracking import Debounce, GapTracker, lane_free_for_manoeuvre

log = logging.getLogger(__name__)

LaneSteerFn = Callable[[carla.Vehicle], float]
ObstacleFn = Callable[[carla.Vehicle], Optional[float]]
# Home waypoint, target offset, lane width, look-behind distance -> free?
# Legacy three-argument plug-ins are also accepted.
AdjacentClearFn = Callable[[carla.Waypoint, float, float, float], bool] | Callable[[carla.Waypoint, float, float], bool]

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


def _nearest_sample(wp: carla.Waypoint, loc: carla.Location, step_m: float,
                    reverse: bool) -> tuple[float, float]:
    sample, travelled = wp, 0.0
    best = (float("inf"), 0.0)  # (distance to sample, arc length)
    while travelled < DETECTION_RANGE and sample is not None:
        dist = sample.transform.location.distance(loc)
        if dist < best[0]:
            best = (dist, travelled)
        sample = _step(sample, step_m, reverse)
        travelled += step_m
    return best


def ground_truth_obstacle_distance(world: carla.World, step_m: float = 1.0,
                                   interpolate: bool = False,
                                   route: Optional[RouteFollower] = None) -> ObstacleFn:
    """Fallback obstacle module (stands in for Abir's LiDAR module).

    Walks the car's lane forward (so it works on curves) and reports the gap to the
    nearest vehicle whose body overlaps the car's predicted path.
    """
    cmap = world.get_map()
    if route is not None:
        return route_obstacle_distance(world, route)

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
            best = _nearest_sample(wp, loc, step_m, reverse)
            arc = best[1]
            anchor = _step(wp, arc, reverse) if arc > 0 else wp
            if anchor is None:
                continue
            lat = _lateral(anchor, loc) * (-1 if reverse else 1)
            overlap = ego_half_w + actor.bounding_box.extent.y + PATH_MARGIN
            if abs(lat - car_lat) < overlap:
                if interpolate:
                    direction = anchor.transform.get_forward_vector()
                    delta = loc - anchor.transform.location
                    projection = delta.x * direction.x + delta.y * direction.y
                    arc += projection * (-1 if reverse else 1)
                gap = arc - actor.bounding_box.extent.x - ego_half_l
                nearest = gap if nearest is None else min(nearest, gap)
        return nearest

    return fn


def route_obstacle_distance(world: carla.World, route: RouteFollower) -> ObstacleFn:
    """Project actor centres onto the route polyline and subtract bumper extents."""
    def fn(vehicle: carla.Vehicle) -> Optional[float]:
        route.update(vehicle)
        nearest = None
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.id == vehicle.id:
                continue
            loc = actor.get_location()
            best = (math.inf, 0.0)
            for i in range(route.index, len(route.route)-1):
                a, b = route.route[i:i+2]
                dx, dy = b.x-a.x, b.y-a.y
                square = dx*dx+dy*dy
                t = max(0.0,min(1.0,((loc.x-a.x)*dx+(loc.y-a.y)*dy)/square)) if square else 0.0
                lateral = math.hypot(loc.x-a.x-t*dx,loc.y-a.y-t*dy)
                arc = route.arc[i]+t*(route.arc[i+1]-route.arc[i])-route.progress_m
                if lateral < best[0] and 0 < arc <= DETECTION_RANGE:
                    best = lateral, arc
            overlap = vehicle.bounding_box.extent.y+actor.bounding_box.extent.y+PATH_MARGIN
            if best[0] < overlap:
                gap = best[1]-vehicle.bounding_box.extent.x-actor.bounding_box.extent.x
                nearest = gap if nearest is None else min(nearest,gap)
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
        tracking_distance_fn: Optional[ObstacleFn] = None,
        home_waypoint: Optional[carla.Waypoint] = None,
        route: Optional[RouteFollower] = None,
    ) -> None:
        self.world = world
        self.map = world.get_map()
        self.vehicle = vehicle
        self.config = config
        self.route = route
        self.lane_steer_fn = lane_steer_fn or (route.steer if route else waypoint_lane_steer(world))
        self.obstacle_distance_fn = obstacle_distance_fn or ground_truth_obstacle_distance(world, route=route)
        if route is not None and hasattr(self.obstacle_distance_fn, "path_fn"):
            self.obstacle_distance_fn.path_fn = route.path_ahead
        self.tracking_distance_fn = tracking_distance_fn or (
            ground_truth_obstacle_distance(world, interpolate=True, route=route)
            if obstacle_distance_fn is None else self.obstacle_distance_fn)
        self.adjacent_clear_fn = adjacent_clear_fn  # None -> ground truth
        self._home_wp: Optional[carla.Waypoint] = home_waypoint
        self.odometer = 0.0
        # Set lazily: before the first world.tick() a new actor reports (0,0,0).
        self._last_location: Optional[carla.Location] = None
        self._obstacle_tracker = GapTracker()
        self._last_frame: Optional[int] = None
        self._adjacent_debounce = Debounce(config.adjacent_clear_hold)
        self._adjacent_frame: Optional[int] = None
        self._adjacent_clear = False

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
        if self.route is not None:
            self.route.update(self.vehicle)
            point = self.route.current
            location = carla.Location(x=point.x,y=point.y,z=point.z)
            return self.map.get_waypoint(location)
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
        if self.route is not None:
            self.route.update(self.vehicle)
            point = self.route.current
            loc = self.vehicle.get_location()
            yaw = math.radians(point.yaw)
            return -(loc.x-point.x)*math.sin(yaw)+(loc.y-point.y)*math.cos(yaw)
        wp = self._home_waypoint()
        d = self.vehicle.get_location() - wp.transform.location
        r = wp.transform.get_right_vector()
        return d.x * r.x + d.y * r.y

    def heading_error(self) -> float:
        """Car yaw minus lane yaw, radians, wrapped to [-pi, pi] (+ = pointing right)."""
        lane_yaw = (self.route.current.yaw if self.route is not None
                    else self._home_waypoint().transform.rotation.yaw)
        car_yaw = self.vehicle.get_transform().rotation.yaw
        diff = math.radians(car_yaw - lane_yaw)
        return math.atan2(math.sin(diff), math.cos(diff))

    def adjacent_lane_clear(self) -> bool:
        """Is the avoid-side lane free? The map must have a driving lane there; whether
        it is occupied comes from the plug-in (LiDAR) or the ground-truth fallback."""
        home = self._home_waypoint()
        adjacent = self._adjacent_wp(home)
        if adjacent is None:
            raw = False
        elif self.adjacent_clear_fn is None:
            raw = self.ground_truth_adjacent_lane_clear()
        else:
            target = self.config.avoid_side * self.config.lane_width
            behind = 0.0 if adjacent.lane_id * home.lane_id < 0 else ADJ_LANE_CHECK_BACK
            args = (home, target, self.config.lane_width, behind)
            try:
                inspect.signature(self.adjacent_clear_fn).bind(*args)
            except TypeError:
                raw = self.adjacent_clear_fn(*args[:3])
            else:
                raw = self.adjacent_clear_fn(*args)
        try:
            snapshot = self.world.get_snapshot()
        except AttributeError:
            snapshot = None
        frame = getattr(snapshot, "frame", None)
        dt = getattr(getattr(snapshot, "timestamp", None), "delta_seconds", 0.05)
        dt = dt if dt is not None and dt > 0 else 0.05
        # Repeated reads in one world tick must not advance the clearance timer.
        if not raw or frame is None or frame != self._adjacent_frame:
            self._adjacent_clear = self._adjacent_debounce.update(raw, dt)
            self._adjacent_frame = frame
        return self._adjacent_clear

    def ground_truth_adjacent_lane_clear(self) -> bool:
        """Fallback (stands in for the LiDAR check): actor positions, measured relative
        to the home lane (road/lane IDs change between road segments, so they can't be
        compared)."""
        home = self._home_waypoint()
        adjacent = self._adjacent_wp(home)
        if adjacent is None:
            return False
        behind = (self.vehicle.bounding_box.extent.x
                  if adjacent.lane_id * home.lane_id < 0 else ADJ_LANE_CHECK_BACK)
        target = self.config.avoid_side * self.config.lane_width
        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.id == self.vehicle.id:
                continue
            loc = actor.get_location()
            fwd, _ = _local_xy(self.vehicle, loc)
            if not -behind < fwd < self.config.adjacent_detection_range:
                continue
            # Measure the actor against the home lane AT THE ACTOR'S POSITION, so the
            # check stays correct on curves (lane direction there differs from ours).
            anchor = _step(home, abs(fwd), reverse=fwd < 0) if abs(fwd) > 0.5 else home
            actor_offset = _lateral(anchor or home, loc)
            if abs(actor_offset - target) <= self.config.lane_width / 2:
                gap = fwd - self.vehicle.bounding_box.extent.x - actor.bounding_box.extent.x
                if gap <= 10.0:
                    return False
                direction = (anchor or home).transform.get_forward_vector()
                velocity = actor.get_velocity()
                other_speed = velocity.x * direction.x + velocity.y * direction.y
                closing = speed_of(self.vehicle) - other_speed
                if not lane_free_for_manoeuvre(gap, closing, self.config.manoeuvre_horizon,
                                              self.config.manoeuvre_margin):
                    return False
        return True

    def perceive(self) -> Perception:
        loc = self.vehicle.get_location()
        if self.route is not None:
            self.route.update(self.vehicle)
        if self._last_location is not None:
            self.odometer += loc.distance(self._last_location)
        self._last_location = loc
        gap = self.obstacle_distance_fn(self.vehicle)
        speed = speed_of(self.vehicle)
        snapshot = self.world.get_snapshot()
        if snapshot.frame != self._last_frame:
            tracked_gap = self.tracking_distance_fn(self.vehicle) if gap is not None else None
            self._obstacle_tracker.update(tracked_gap, speed, snapshot.timestamp.delta_seconds)
            self._last_frame = snapshot.frame
        return Perception(
            lane_steer=self.lane_steer_fn(self.vehicle),
            obstacle_distance=gap,
            obstacle_speed=self._obstacle_tracker.speed,
            dt=snapshot.timestamp.delta_seconds,
            speed=speed,
            adjacent_lane_clear=self.adjacent_lane_clear(),
            lateral_offset=self.lateral_offset(),
            distance_travelled=self.odometer,
            heading_error=self.heading_error(),
            speed_limit=self.route.speed_limit() if self.route else None,
            near_junction=(self.route.next_junction_distance() is not None
                           and self.route.next_junction_distance() < 25) if self.route else False,
        )

    def apply(self, cmd: ControlCommand) -> None:
        if cmd.mode == Mode.CRUISE and abs(self.lateral_offset()) > self.config.lane_width / 2:
            home = self._home_waypoint()
            current = self.map.get_waypoint(self.vehicle.get_location())
            if current.lane_id * home.lane_id >= 0:
                self.reanchor_home_lane()
        self.vehicle.apply_control(
            carla.VehicleControl(throttle=cmd.throttle, steer=cmd.steer, brake=cmd.brake)
        )
