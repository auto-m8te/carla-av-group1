"""Progress tracking and route pursuit using plain geometry."""
from __future__ import annotations
import bisect
import logging
import math
from typing import Any, NamedTuple, Optional
from navigation.route_planner import RoutePoint, distance

log = logging.getLogger(__name__)


STEER_MODES = ("stanley", "pursuit")
STANLEY_GAIN = 1.0            # cross-track gain (1/s), softened by (speed + 1)
STANLEY_SOFTENING = 1.0       # m/s added to speed so the gain stays finite at standstill
FRONT_AXLE_M = 1.4            # car centre to front axle (Tesla Model 3 wheelbase 2.9 m / 2)
WHEELBASE_M = 2.9
CURVATURE_WINDOW_M = 5.0      # route length over which the curve feed-forward is measured
DEFAULT_MAX_STEER_DEG = 70.0  # CARLA wheel angle at steer = 1.0 when physics are unavailable


class _Point(NamedTuple):
    x: float
    y: float
    z: float


def _wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class RouteFollower:
    def __init__(self, route: list[RoutePoint], lookahead_m: float = 6.0,
                 steer_mode: str = "stanley") -> None:
        if not route or lookahead_m <= 0:
            raise ValueError('A nonempty route and positive lookahead are required')
        if steer_mode not in STEER_MODES:
            raise ValueError(f'steer_mode must be one of {STEER_MODES}')
        self.route = tuple(route)
        self.lookahead_m = lookahead_m
        self.steer_mode = steer_mode
        self._max_steer_rad: Optional[float] = None
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

    @staticmethod
    def _speed(vehicle: Any) -> float:
        v = vehicle.get_velocity() if hasattr(vehicle, "get_velocity") else None
        return math.hypot(v.x, v.y) if v is not None else 8.0

    def _max_steer(self, vehicle: Any) -> float:
        """Wheel angle (rad) at steer = 1.0, read once from the vehicle's physics."""
        if self._max_steer_rad is None:
            degrees = DEFAULT_MAX_STEER_DEG
            try:
                degrees = float(vehicle.get_physics_control().wheels[0].max_steer_angle) or degrees
            except (AttributeError, IndexError, RuntimeError, TypeError, ValueError):
                pass
            self._max_steer_rad = math.radians(degrees)
        return self._max_steer_rad

    def curvature(self) -> float:
        """Signed route curvature (1/m, + = turning right) over the next few metres."""
        end = bisect.bisect_left(self.arc, self.progress_m + CURVATURE_WINDOW_M)
        if end >= len(self.route) or end == self.index:
            return 0.0
        turn = math.radians(_wrap_deg(self.route[end].yaw - self.current.yaw))
        return turn / max(self.arc[end] - self.progress_m, 1e-3)

    def steer(self, vehicle: Any) -> float:
        self.update(vehicle)
        if self.steer_mode == "stanley":
            return self._stanley(vehicle)
        return self._pursuit(vehicle)

    def _stanley(self, vehicle: Any) -> float:
        """Stanley controller at the front axle plus curve feed-forward (adopted from
        Prithiv's Lanebot control_node). Pure pursuit aims at a point ahead and therefore
        cuts corners; Stanley steers on the cross-track error at the front axle, and the
        feed-forward term turns the wheels by the angle the curve itself needs."""
        speed = self._speed(vehicle)
        yaw = math.radians(vehicle.get_transform().rotation.yaw)
        loc = vehicle.get_location()
        front = _Point(loc.x + FRONT_AXLE_M * math.cos(yaw),
                       loc.y + FRONT_AXLE_M * math.sin(yaw), getattr(loc, "z", 0.0))
        low = max(0, self.index - 2)
        high = min(len(self.route), bisect.bisect_right(self.arc, self.progress_m + 10.0) + 1)
        nearest = min(range(low, max(low + 1, high)), key=lambda i: distance(self.route[i], front))
        point = self.route[nearest]
        # Signed cross-track error: path position to the right of the front axle (+).
        path_yaw = math.radians(point.yaw)
        dx, dy = point.x - front.x, point.y - front.y
        cross = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        heading = math.radians(_wrap_deg(point.yaw - math.degrees(yaw)))
        wheel = (heading
                 + math.atan2(STANLEY_GAIN * cross, speed + STANLEY_SOFTENING)
                 + math.atan(WHEELBASE_M * self.curvature()))
        return max(-1.0, min(1.0, wheel / self._max_steer(vehicle)))

    def _pursuit(self, vehicle: Any) -> float:
        # Speed-dependent look-ahead (pure pursuit): a fixed 6 m cut corners by up to ~2 m at
        # junction speeds; 2 m + 0.5 s of travel gives ~4 m at 4-5 m/s, 6 m at cruise speed.
        speed = self._speed(vehicle)
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
