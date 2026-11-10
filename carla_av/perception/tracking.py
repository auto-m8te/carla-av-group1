"""Gap-based velocity estimation and conservative lane clearance, without CARLA."""

from __future__ import annotations

import logging
from typing import Optional

log = logging.getLogger(__name__)


class GapTracker:
    """Estimate signed object velocity from consecutive bumper gaps."""

    def __init__(self, alpha: float = 0.3) -> None:
        self.alpha = alpha
        self._gap: Optional[float] = None
        self._estimate: Optional[float] = None
        self._samples = 0

    @property
    def speed(self) -> Optional[float]:
        return self._estimate if self._samples >= 3 else None

    def update(self, gap: Optional[float], ego_speed: float, dt: float) -> None:
        if dt <= 0:
            raise ValueError("dt must be positive")
        if gap is None or self._gap is None or abs(gap - self._gap) > 3.0:
            self._estimate = None
            self._samples = 0 if gap is None else 1
        else:
            measured = ego_speed + (gap - self._gap) / dt
            self._estimate = (measured if self._estimate is None else
                              self.alpha * measured + (1 - self.alpha) * self._estimate)
            self._samples += 1
        self._gap = gap


def lane_free_for_manoeuvre(
    nearest_ahead: Optional[float], closing_speed: Optional[float],
    horizon_s: float, margin_m: float,
) -> bool:
    """Visible traffic must remain beyond the margin throughout the manoeuvre."""
    if nearest_ahead is None:
        return True
    if closing_speed is None:
        return False
    return nearest_ahead - max(closing_speed, 0.0) * horizon_s > margin_m


class Debounce:
    """Block immediately; release only after continuously clear elapsed time."""

    def __init__(self, hold: float = 0.5) -> None:
        self.hold = hold
        self._clear_time = 0.0

    def update(self, clear: bool, dt: float) -> bool:
        if not clear:
            self._clear_time = 0.0
            return False
        self._clear_time += dt
        return self._clear_time + 1e-12 >= self.hold
