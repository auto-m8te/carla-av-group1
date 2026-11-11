"""Directed topology routing without simulator or graph-library dependencies."""
from __future__ import annotations
from dataclasses import dataclass
import heapq
import itertools
import logging
import math
from typing import Any, Hashable, Iterable

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class RoutePoint:
    x: float
    y: float
    z: float
    yaw: float
    is_junction: bool
    road_id: int
    lane_id: int


def distance(a: Any, b: Any) -> float:
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)


def shortest_path(nodes: Iterable[Hashable], edges: Iterable[tuple],
                  src: Hashable, dst: Hashable) -> list:
    """Return the least-cost directed path, or [] when unreachable."""
    graph = {n: [] for n in nodes}
    for a, b, cost in edges:
        if cost < 0:
            raise ValueError('Dijkstra requires nonnegative costs')
        graph.setdefault(a, []).append((b, cost))
    serial = itertools.count()
    queue = [(0.0, next(serial), src)]
    costs, previous = {src: 0.0}, {}
    while queue:
        cost, _, node = heapq.heappop(queue)
        if cost != costs[node]:
            continue
        if node == dst:
            path = [dst]
            while path[-1] != src:
                path.append(previous[path[-1]])
            return path[::-1]
        for target, weight in graph.get(node, []):
            candidate = cost + weight
            if candidate < costs.get(target, math.inf):
                costs[target], previous[target] = candidate, node
                heapq.heappush(queue, (candidate, next(serial), target))
    return []


def route_point(wp: Any) -> RoutePoint:
    loc = wp.transform.location
    return RoutePoint(loc.x, loc.y, loc.z, wp.transform.rotation.yaw,
                      wp.is_junction, wp.road_id, wp.lane_id)


def sample_segment(start: Any, end: Any, step_m: float) -> list[Any]:
    samples, current, seen = [start], start, set()
    for _ in range(20000):
        if distance(current.transform.location, end.transform.location) <= step_m:
            return samples + [end]
        key = (current.road_id, current.lane_id, round(current.s, 3))
        if key in seen:
            break
        seen.add(key)
        choices = current.next(step_m)
        if not choices:
            break
        current = min(choices, key=lambda w: distance(w.transform.location, end.transform.location))
        if (current.road_id, current.lane_id) != (start.road_id, start.lane_id):
            break
        samples.append(current)
    # Sampling can stop early (the next waypoint hops to another road id right at a
    # segment boundary, or a loop); keep what we have and close the segment at its end
    # instead of failing the whole map (that made planning fail on 5 of 6 towns).
    return samples + [end]


def _lane_change(a: Any, b: Any) -> bool:
    if a.is_junction or b.is_junction or a.road_id != b.road_id:
        return False
    if a.lane_id == b.lane_id or a.lane_id * b.lane_id <= 0:
        return False
    yaw = math.radians(a.transform.rotation.yaw-b.transform.rotation.yaw)
    driving = str(getattr(b, 'lane_type', 'Driving')).endswith('Driving')
    return driving and math.cos(yaw) > 0.95 and abs(a.s-b.s) < 2.0


def plan_route(cmap: Any, start_location: Any, goal_location: Any,
               step_m: float = 2.0) -> list[RoutePoint]:
    if step_m <= 0:
        raise ValueError('step_m must be positive')
    segments = [sample_segment(a, b, step_m) for a,b in cmap.get_topology()]
    points = [route_point(w) for segment in segments for w in segment]
    waypoints = [w for segment in segments for w in segment]
    edges, starts, ends, offset = [], [], [], 0
    for segment in segments:
        starts.append(offset)
        for i in range(len(segment)-1):
            edges.append((offset+i, offset+i+1, distance(points[offset+i], points[offset+i+1])))
        offset += len(segment)
        ends.append(offset-1)
    for end in ends:
        for start in starts:
            if distance(points[end], points[start]) <= 0.5:
                edges.append((end, start, distance(points[end], points[start])))
    buckets: dict[tuple, list[int]] = {}
    for i, wp in enumerate(waypoints):
        key = (wp.road_id, round(wp.s / step_m))
        for j in buckets.get(key, []):
            if _lane_change(wp, waypoints[j]):
                cost = distance(points[i], points[j]) + 30.0
                edges.extend([(i,j,cost),(j,i,cost)])
        buckets.setdefault(key, []).append(i)
    if not points:
        return []
    def nearest(loc: Any) -> int:
        wp = cmap.get_waypoint(loc)
        eligible = [i for i,p in enumerate(points) if (p.road_id,p.lane_id)==(wp.road_id,wp.lane_id)]
        return min(eligible or range(len(points)), key=lambda i: distance(points[i],loc))
    return [points[i] for i in shortest_path(range(len(points)), edges, nearest(start_location), nearest(goal_location))]
