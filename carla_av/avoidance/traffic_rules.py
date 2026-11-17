"""Speed rules adopted from Prithiv's Lanebot control_node (StVO §3).

  * drive at most 90 % of the signed speed limit,
  * be down to a lower limit by the time we reach its sign,
  * "Fahren auf Sicht": only as fast as we can stop within what the sensors can see.

Pure functions, so they are unit-tested without CARLA.
"""
from __future__ import annotations

import math
from typing import Optional

LIMIT_SHARE = 0.9        # drive at 90 % of the limit
SIGN_DECEL = 3.0         # m/s^2, comfortable braking to reach a lower limit
SIGN_MARGIN_M = 5.0      # be at the new speed this far before the sign
SIGHT_DECEL = 4.0        # m/s^2, firm braking assumed for "Fahren auf Sicht"
SPEEDING_TOLERANCE_KMH = 3.0   # Prithiv's monitor: speeding = more than 3 km/h over


def kmh(speed_mps: float) -> float:
    return speed_mps * 3.6


def sight_speed(sensor_range_m: float, standstill_gap_m: float,
                decel: float = SIGHT_DECEL) -> float:
    """Fastest speed from which we can stop within the visible distance."""
    return math.sqrt(2.0 * decel * max(0.0, sensor_range_m - standstill_gap_m))


def sign_speed(new_limit_kmh: float, distance_m: float, share: float = LIMIT_SHARE,
               decel: float = SIGN_DECEL, margin_m: float = SIGN_MARGIN_M) -> float:
    """Highest speed now that still lets us slow to the new limit before its sign."""
    target = share * new_limit_kmh / 3.6
    return math.sqrt(target * target + 2.0 * decel * max(0.0, distance_m - margin_m))


def rule_speed_limit(limit_kmh: Optional[float], next_sign: Optional[tuple[float, float]],
                     sensor_range_m: float, standstill_gap_m: float,
                     share: float = LIMIT_SHARE) -> Optional[float]:
    """Speed cap in m/s from the three rules, or None when nothing is known.

    limit_kmh: signed limit here; next_sign: (lower limit km/h, metres to the sign)."""
    caps = [sight_speed(sensor_range_m, standstill_gap_m)]
    if limit_kmh and limit_kmh > 0:
        caps.append(share * limit_kmh / 3.6)
    if next_sign is not None:
        caps.append(sign_speed(*next_sign, share=share))
    return min(caps)


def is_speeding(speed_mps: float, limit_kmh: Optional[float],
                tolerance_kmh: float = SPEEDING_TOLERANCE_KMH) -> bool:
    return bool(limit_kmh) and kmh(speed_mps) > limit_kmh + tolerance_kmh
