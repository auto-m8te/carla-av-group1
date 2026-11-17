"""Stop at red and amber traffic lights along the planned route.

Adopted from Prithiv's Lanebot (signals_node / control_node, StVO §37): look up every
traffic light's stop line on our route; at red always stop before it, at amber stop
only if a comfortable stop is still possible, otherwise drive through.

The controller already knows how to stop for a parked car, so a red light is handed
to it as a stationary "obstacle" at the stop line (and overtaking is disabled there).
"""
from __future__ import annotations

import bisect
import logging
from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence

log = logging.getLogger(__name__)

GREEN, AMBER, RED = 0, 1, 2
STOP_LINE_MATCH_M = 3.0     # a stop waypoint belongs to the route if this close to a route point
LOOKAHEAD_M = 80.0          # look for stop lines this far ahead (enough to stop from 50 km/h)
AMBER_DECEL = 4.0           # m/s^2: at amber, stop only if this deceleration is enough


@dataclass(frozen=True)
class StopLine:
    arc_m: float                       # position of the stop line along the route
    state_fn: Callable[[], int]        # current light state: GREEN / AMBER / RED


def light_stop_gap(stop_lines: Sequence[StopLine], progress_m: float, speed: float,
                   bumper_m: float, lookahead_m: float = LOOKAHEAD_M,
                   amber_decel: float = AMBER_DECEL) -> Optional[float]:
    """Bumper gap (m) to the first stop line we must stop at, or None.

    Lines behind the front bumper are ignored: once the nose is over the line we are
    committed and must clear the junction.
    """
    for line in stop_lines:
        gap = line.arc_m - progress_m - bumper_m
        if gap < 0.0:
            continue
        if gap > lookahead_m:
            return None
        state = line.state_fn()
        if state == RED:
            return gap
        if state == AMBER and speed * speed / (2.0 * amber_decel) < gap:
            return gap
    return None


def match_stop_lines(route_points: Sequence[Any], arc: Sequence[float],
                     stops: Sequence[tuple[Any, Callable[[], int]]],
                     match_m: float = STOP_LINE_MATCH_M) -> list[StopLine]:
    """Place each (stop waypoint location + road/lane ids, state getter) on the route.

    `stops` items are (waypoint, state_fn); a waypoint matches a route point on the same
    road and lane that lies within `match_m`. Returns the lines sorted along the route.
    """
    lines: dict[float, StopLine] = {}
    for wp, state_fn in stops:
        loc = wp.transform.location
        best: Optional[tuple[float, int]] = None
        for i, p in enumerate(route_points):
            if p.road_id != wp.road_id or p.lane_id != wp.lane_id:
                continue
            d = ((p.x - loc.x) ** 2 + (p.y - loc.y) ** 2) ** 0.5
            if d <= match_m and (best is None or d < best[0]):
                best = (d, i)
        if best is not None:
            lines.setdefault(arc[best[1]], StopLine(arc[best[1]], state_fn))
    return [lines[k] for k in sorted(lines)]


class RouteTrafficLights:
    """Traffic lights on one route, with red-light violation counting for evaluation."""

    def __init__(self, stop_lines: Sequence[StopLine]) -> None:
        self.stop_lines = tuple(stop_lines)
        self._arcs = [line.arc_m for line in self.stop_lines]
        self._last_nose: Optional[float] = None
        self.violations = 0

    @classmethod
    def from_world(cls, world: Any, route: Any) -> "RouteTrafficLights":
        import carla  # local import: the rest of this module is testable without CARLA
        codes = {carla.TrafficLightState.Red: RED, carla.TrafficLightState.Yellow: AMBER}
        stops = []
        for light in world.get_actors().filter("traffic.traffic_light"):
            def state_fn(light=light) -> int:
                return codes.get(light.get_state(), GREEN)
            stops.extend((wp, state_fn) for wp in light.get_stop_waypoints())
        lines = match_stop_lines(route.route, route.arc, stops)
        log.info("Route has %d traffic-light stop lines", len(lines))
        return cls(lines)

    def gap(self, progress_m: float, speed: float, bumper_m: float) -> Optional[float]:
        self._count_violations(progress_m, bumper_m)
        return light_stop_gap(self.stop_lines, progress_m, speed, bumper_m)

    def _count_violations(self, progress_m: float, bumper_m: float) -> None:
        """A violation: the front bumper crosses a stop line while it is red."""
        nose = progress_m + bumper_m
        if self._last_nose is not None and nose > self._last_nose:
            lo = bisect.bisect_right(self._arcs, self._last_nose)
            hi = bisect.bisect_right(self._arcs, nose)
            self.violations += sum(self.stop_lines[i].state_fn() == RED for i in range(lo, hi))
        self._last_nose = nose
