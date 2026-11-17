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
from navigation.traffic_lights import RouteTrafficLights
from avoidance.traffic_rules import rule_speed_limit
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
LANE_CONTINUE_STEP = 5.0   # m between checks that the target lane still exists ahead
BROKEN_MARKINGS = ("Broken", "BrokenBroken", "BrokenSolid", "NONE")
BLINKER_MODES = {Mode.AVOID: "out", Mode.PASSING: "out", Mode.RETURN: "back", Mode.ABORT: "back",
                 Mode.YIELD: "right"}
EMERGENCY_KINDS = {"ambulance": "Ambulance", "police": "Police", "firetruck": "Fire engine"}
EMERGENCY_SIDE_M = 8.0      # sideways: further than this is the other side of the road
PEDESTRIAN_AHEAD_M = 45.0   # pedestrians this far ahead (and near the road) block overtaking
PEDESTRIAN_SIDE_M = 8.0
SPEED_SIGN_LOOKAHEAD_M = 100.0   # look for lower speed-limit signs this far ahead
SPEED_SIGN_TYPE = "274"          # OpenDRIVE / StVO code of a speed-limit sign
DIP_RANGE_M = 200.0              # dip the high beam for vehicles this close in front
DIP_SIDE_M = 30.0


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
        actors = list(world.get_actors().filter("vehicle.*")) + list(world.get_actors().filter("walker.*"))
        others = [a for a in actors
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


def _min_limit(*limits: Optional[float]) -> Optional[float]:
    known = [v for v in limits if v is not None]
    return min(known) if known else None


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
        traffic_lights: Optional[RouteTrafficLights] = None,
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
        self.traffic_lights = traffic_lights
        self.light_gap: Optional[float] = None   # bumper gap to a red/amber stop line
        self.raw_obstacle_distance: Optional[float] = None  # perception only, without lights
        self._manoeuvre_length = 0.0   # m the target lane must continue (set each tick)
        self._blinker: Optional[str] = None
        self.emergency_kind: Optional[str] = None   # for the instrument cluster
        self.emergency_distance: Optional[float] = None
        self.pedestrian_near = False
        self.map_speed_limit_kmh: Optional[float] = None   # signed limit here
        self.next_speed_sign: Optional[tuple[float, float]] = None
        self.night = False                                  # headlights: dip / high beam
        self._lights: Optional[tuple] = None

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

    def lane_change_allowed(self, home: carla.Waypoint) -> bool:
        """StVO §5: cross only a broken line, where the map permits a lane change."""
        if not self.config.obey_lane_markings:
            return True
        left = self.config.avoid_side < 0
        marking = home.left_lane_marking if left else home.right_lane_marking
        if str(getattr(marking, "type", "NONE")) not in BROKEN_MARKINGS:
            return False
        allowed = (carla.LaneChange.Left if left else carla.LaneChange.Right, carla.LaneChange.Both)
        return home.lane_change in allowed

    def target_lane_continues(self, home: carla.Waypoint, length: float) -> bool:
        """The target lane must exist for the whole manoeuvre: a lane that ends at a merge
        or turns into a parking lane would leave us stranded beside the obstacle
        (Prithiv's left_room check)."""
        start = self._adjacent_wp(home)
        if start is None:
            return False
        same_direction = start.lane_id * home.lane_id > 0
        travelled, wp = 0.0, home
        while travelled < length:
            wp = _step(wp, LANE_CONTINUE_STEP, reverse=False)
            if wp is None:
                return False
            travelled += LANE_CONTINUE_STEP
            adjacent = self._adjacent_wp(wp)
            if adjacent is None or (adjacent.lane_id * wp.lane_id > 0) != same_direction:
                return False
        return True

    def adjacent_lane_clear(self) -> bool:
        """Is the avoid-side lane free? The map must have a driving lane there that we may
        change into and that continues for the manoeuvre; whether it is occupied comes from
        the plug-in (LiDAR) or the ground-truth fallback."""
        home = self._home_waypoint()
        adjacent = self._adjacent_wp(home)
        if (adjacent is None or not self.lane_change_allowed(home)
                or not self.target_lane_continues(home, self._manoeuvre_length)):
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

    def _actors_around(self, pattern: str) -> list[tuple[carla.Actor, float, float]]:
        """(actor, metres ahead, metres to the right) for every actor matching pattern."""
        out = []
        try:
            actors = list(self.world.get_actors().filter(pattern))
        except AttributeError:   # unit-test worlds without actors
            return out
        for actor in actors:
            if actor.id == self.vehicle.id:
                continue
            ahead, right = _local_xy(self.vehicle, actor.get_location())
            out.append((actor, ahead, right))
        return out

    def nearest_emergency_vehicle(self) -> tuple[Optional[float], Optional[str]]:
        """Signed distance (+ ahead, - behind) and kind of the nearest emergency vehicle on
        our side of the road. Like Prithiv's signals_node, this reads the vehicle type from
        the simulator (a real car would hear the siren / see the blue lights)."""
        best: tuple[Optional[float], Optional[str]] = (None, None)
        for actor, ahead, right in self._actors_around("vehicle.*"):
            type_id = getattr(actor, "type_id", "")
            kind = next((name for key, name in EMERGENCY_KINDS.items() if key in type_id), None)
            if kind and abs(right) < EMERGENCY_SIDE_M and (best[0] is None or abs(ahead) < abs(best[0])):
                best = (ahead, kind)
        return best

    def pedestrian_ahead(self) -> bool:
        """A pedestrian close ahead near the road: overtaking is not allowed (StVO §26 spirit).
        Detection and stopping still come from the LiDAR; this only blocks the lane change."""
        return any(0.0 < ahead < PEDESTRIAN_AHEAD_M and abs(right) < PEDESTRIAN_SIDE_M
                   for _, ahead, right in self._actors_around("walker.pedestrian.*"))

    def speed_rules(self) -> Optional[float]:
        """Speed cap (m/s) from the signed limit, the next lower sign and the sensor
        range (Prithiv's control_node target_speed, StVO §3)."""
        try:
            self.map_speed_limit_kmh = float(self.vehicle.get_speed_limit()) or None
        except (AttributeError, RuntimeError, TypeError):
            self.map_speed_limit_kmh = None
        self.next_speed_sign = None
        try:
            wp = self.map.get_waypoint(self.vehicle.get_location())
            for sign in wp.get_landmarks_of_type(SPEED_SIGN_LOOKAHEAD_M, SPEED_SIGN_TYPE, False):
                lower = self.map_speed_limit_kmh is None or sign.value < self.map_speed_limit_kmh
                if lower and (self.next_speed_sign is None or sign.distance < self.next_speed_sign[1]):
                    self.next_speed_sign = (float(sign.value), float(sign.distance))
        except (AttributeError, RuntimeError, TypeError):
            pass
        if not self.config.obey_speed_limits:
            return None
        return rule_speed_limit(self.map_speed_limit_kmh, self.next_speed_sign,
                                self.config.adjacent_detection_range, self.config.standstill_gap)

    def perceive(self) -> Perception:
        loc = self.vehicle.get_location()
        if self.route is not None:
            self.route.update(self.vehicle)
        if self._last_location is not None:
            self.odometer += loc.distance(self._last_location)
        self._last_location = loc
        gap = self.obstacle_distance_fn(self.vehicle)
        self.raw_obstacle_distance = gap
        speed = speed_of(self.vehicle)
        snapshot = self.world.get_snapshot()
        if snapshot.frame != self._last_frame:
            tracked_gap = self.tracking_distance_fn(self.vehicle) if gap is not None else None
            self._obstacle_tracker.update(tracked_gap, speed, snapshot.timestamp.delta_seconds)
            self._last_frame = snapshot.frame
        obstacle_speed = self._obstacle_tracker.speed
        # Distance the target lane must continue: up to the obstacle, past it, and back.
        self._manoeuvre_length = ((gap or 0.0) + self.config.pass_clear_distance
                                  + speed * self.config.lane_change_time
                                  + self.config.lane_continue_extra)
        adjacent_clear = self.adjacent_lane_clear()
        self.light_gap = None
        if self.traffic_lights is not None and self.route is not None:
            self.light_gap = self.traffic_lights.gap(self.route.progress_m, speed,
                                                     self.config.ego_bumper_margin)
            # A red light is a stationary obstacle at the stop line: stop, never overtake it.
            # The controller keeps safety_margin to an obstacle; a stop line may be approached
            # closer, so it is reported light_stop_shift metres further away.
            light = None if self.light_gap is None else self.light_gap + self.config.light_stop_shift
            if light is not None and (gap is None or light < gap):
                gap, obstacle_speed, adjacent_clear = light, 0.0, False
        self.emergency_distance, self.emergency_kind = self.nearest_emergency_vehicle()
        self.pedestrian_near = self.pedestrian_ahead()
        return Perception(
            lane_steer=self.lane_steer_fn(self.vehicle),
            obstacle_distance=gap,
            obstacle_speed=obstacle_speed,
            dt=snapshot.timestamp.delta_seconds,
            speed=speed,
            adjacent_lane_clear=adjacent_clear,
            lateral_offset=self.lateral_offset(),
            distance_travelled=self.odometer,
            heading_error=self.heading_error(),
            speed_limit=_min_limit(self.route.speed_limit() if self.route else None,
                                   self.speed_rules()),
            emergency_distance=self.emergency_distance,
            pedestrian_near=self.pedestrian_near,
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
        self._set_indicator(BLINKER_MODES.get(cmd.mode))

    def high_beam_allowed(self) -> bool:
        """At night use high beam unless a vehicle is in front of us or coming towards
        us within 200 m (Prithiv's signals_node)."""
        if not self.night:
            return False
        return not any(0.0 < ahead < DIP_RANGE_M and abs(right) < DIP_SIDE_M
                       for _, ahead, right in self._actors_around("vehicle.*"))

    def _set_indicator(self, direction: Optional[str]) -> None:
        """Indicator on while pulling out or moving back (StVO §5/§7, from Prithiv's
        signals_node), plus dipped / high beam at night. Only sent when it changes."""
        high = self.high_beam_allowed()
        if (direction, high) == self._lights:
            return
        self._blinker, self._lights = direction, (direction, high)
        lights = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
        if high:
            lights |= carla.VehicleLightState.HighBeam
        if direction is not None:
            out_left = self.config.avoid_side < 0
            left = {"out": out_left, "back": not out_left, "right": False}[direction]
            lights |= (carla.VehicleLightState.LeftBlinker if left
                       else carla.VehicleLightState.RightBlinker)
        try:
            self.vehicle.set_light_state(carla.VehicleLightState(lights))
        except (AttributeError, RuntimeError) as exc:
            log.debug("Could not set indicator: %s", exc)
