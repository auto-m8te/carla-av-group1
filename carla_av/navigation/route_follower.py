"""Progress tracking and route pursuit using plain geometry."""
from __future__ import annotations
import bisect
import logging
import math
from typing import Any, Optional
from navigation.route_planner import RoutePoint, distance

log = logging.getLogger(__name__)


class RouteFollower:
    def __init__(self, route: list[RoutePoint], lookahead_m: float = 6.0) -> None:
        if not route or lookahead_m <= 0:
            raise ValueError('A nonempty route and positive lookahead are required')
        self.route = tuple(route)
        self.lookahead_m = lookahead_m
        self.arc = [0.0]
        for a,b in zip(route,route[1:]):
            self.arc.append(self.arc[-1]+distance(a,b))
        self.index = 0
        self._furthest = 0.0
        self._goal_distance = math.inf

    def update(self, vehicle: Any) -> None:
        loc = vehicle.get_location()
        low = bisect.bisect_left(self.arc, self._furthest-5.0)
        # A local window prevents jumping to a later crossing of the same road.
        high = bisect.bisect_right(self.arc, self._furthest+60.0)
        self.index = min(range(low, max(low+1,high)), key=lambda i: distance(self.route[i],loc))
        self._furthest = max(self._furthest,self.arc[self.index])
        self._goal_distance = distance(self.route[-1],loc)

    @property
    def progress_m(self) -> float:
        return self.arc[self.index]

    @property
    def remaining_m(self) -> float:
        return self.arc[-1]-self.progress_m

    @property
    def done(self) -> bool:
        return self.remaining_m <= 5 and self._goal_distance <= 5

    @property
    def current(self) -> RoutePoint:
        return self.route[self.index]

    def _local(self, vehicle: Any, point: RoutePoint) -> tuple[float,float]:
        loc = vehicle.get_location()
        yaw = math.radians(vehicle.get_transform().rotation.yaw)
        dx,dy = point.x-loc.x,point.y-loc.y
        return dx*math.cos(yaw)+dy*math.sin(yaw), -dx*math.sin(yaw)+dy*math.cos(yaw)

    def steer(self, vehicle: Any) -> float:
        self.update(vehicle)
        # Speed-dependent look-ahead (pure pursuit): a fixed 6 m cut corners by up to ~2 m at
        # junction speeds; 2 m + 0.5 s of travel gives ~4 m at 4-5 m/s, 6 m at cruise speed.
        v = vehicle.get_velocity() if hasattr(vehicle, "get_velocity") else None
        speed = math.hypot(v.x, v.y) if v is not None else 8.0
        lookahead = min(self.lookahead_m, max(3.0, 2.0 + 0.5 * speed))
        target = min(len(self.route)-1,bisect.bisect_left(self.arc,self.progress_m+lookahead))
        forward,right = self._local(vehicle,self.route[target])
        return max(-1.0,min(1.0,math.atan2(right,max(forward,0.1))/math.radians(35)))

    def path_ahead(self, vehicle: Any, length: float) -> list[tuple[float,float]]:
        self.update(vehicle)
        end = min(len(self.route),bisect.bisect_right(self.arc,self.progress_m+length)+1)
        # The LiDAR corridor is indexed by forward distance, so it can only describe the
        # route while that keeps increasing; in a sharp turn the points after the corner
        # share the same x and would bend the corridor into the corner buildings. Stop
        # the path where it turns away (the corridor then ends there, see PATH_END_MARGIN).
        path: list[tuple[float, float]] = []
        for point in self.route[self.index:end]:
            fwd, right = self._local(vehicle, point)
            if path and fwd <= path[-1][0] + 0.1:
                if fwd < path[-1][0] - 1.0:
                    break
                continue
            path.append((fwd, right))
        return path

    def next_junction_distance(self) -> Optional[float]:
        return next((self.arc[i]-self.progress_m for i in range(self.index,len(self.route))
                     if self.route[i].is_junction),None)

    def speed_limit(self) -> Optional[float]:
        end = bisect.bisect_right(self.arc,self.progress_m+15)
        for point in self.route[self.index:end]:
            delta = (point.yaw-self.current.yaw+180)%360-180
            if abs(delta)>45:
                return 4.0
        junction = self.next_junction_distance()
        return 5.0 if junction is not None and junction < 20 else None
