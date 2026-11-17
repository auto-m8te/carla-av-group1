"""Per-tick traffic-rule checks, adopted from Prithiv's tools/monitor.py.

Each tick it records whether our car is speeding (more than 3 km/h over the signed
limit), off the road (not on a driving lane), or in a near miss (another road user
within 1 m of our car's bumpers). Red-light crossings are counted by
navigation/traffic_lights.py. The analysis turns these into a rule-check table.
"""
from __future__ import annotations

import math
from typing import Optional

import carla

from avoidance.traffic_rules import is_speeding

NEAR_MISS_M = 1.0          # bumper gap counted as a near miss
SIDE_OVERLAP_M = 2.2       # other road user is in line with us if this close sideways
SEAM_PROBE_M = 0.3         # CARLA lane maps have thin seams on lane lines: probe either side


def off_road(cmap: carla.Map, vehicle: carla.Actor) -> bool:
    """True when the car is not on a driving lane (pavement, shoulder, grass)."""
    loc = vehicle.get_location()
    wp = cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
    if wp is None:
        right = vehicle.get_transform().get_right_vector()
        for d in (-SEAM_PROBE_M, SEAM_PROBE_M):
            wp = wp or cmap.get_waypoint(loc + carla.Location(x=right.x * d, y=right.y * d),
                                         project_to_road=False, lane_type=carla.LaneType.Any)
    return wp is None or wp.lane_type != carla.LaneType.Driving


def nearest_in_line_gap(world: carla.World, vehicle: carla.Actor) -> Optional[float]:
    """Smallest bumper gap to any vehicle or pedestrian in line with our car (ahead or behind)."""
    tf = vehicle.get_transform()
    loc, fwd = tf.location, tf.get_forward_vector()
    best: Optional[float] = None
    actors = list(world.get_actors().filter("vehicle.*")) + list(world.get_actors().filter("walker.*"))
    for other in actors:
        if other.id == vehicle.id:
            continue
        d = other.get_location() - loc
        ahead = d.x * fwd.x + d.y * fwd.y
        side = math.sqrt(max(0.0, d.x * d.x + d.y * d.y - ahead * ahead))
        if side < SIDE_OVERLAP_M:
            gap = abs(ahead) - vehicle.bounding_box.extent.x - other.bounding_box.extent.x
            best = gap if best is None else min(best, gap)
    return best


def rule_values(world: carla.World, vehicle: carla.Actor, speed: float,
                limit_kmh: Optional[float]) -> dict:
    """CSV columns for one tick."""
    gap = nearest_in_line_gap(world, vehicle)
    return {
        "speed_limit_map": "" if not limit_kmh else limit_kmh,
        "speeding": int(is_speeding(speed, limit_kmh)),
        "off_road": int(off_road(world.get_map(), vehicle)),
        "near_miss": int(gap is not None and gap < NEAR_MISS_M),
    }
