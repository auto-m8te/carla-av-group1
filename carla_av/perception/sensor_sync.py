"""Frame-synchronised reads of CARLA sensor queues (synchronous mode).

Sensor data for a tick arrives shortly AFTER world.tick() returns. Reading only what is
already queued can return data several frames old when the simulator is loaded, which
delays reactions. These helpers wait (bounded) for the measurement of the current frame.
"""

from __future__ import annotations

import queue
import time
from typing import Any, Optional


def wait_for_frame(q: "queue.Queue[Any]", frame: int, timeout_s: float = 1.0) -> Optional[Any]:
    """Newest queued item, waiting until one with `.frame >= frame` arrives or timeout.

    Returns None only if nothing new arrived at all (e.g. the very first tick)."""
    newest = None
    deadline = time.monotonic() + timeout_s
    while True:
        try:  # drain everything already queued, keep the newest
            newest = q.get_nowait()
            continue
        except queue.Empty:
            pass
        if frame < 0 or (newest is not None and newest.frame >= frame):
            return newest
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return newest
        try:
            newest = q.get(timeout=remaining)
        except queue.Empty:
            return newest


def current_frame(world: Any) -> int:
    """World frame number, or -1 (= do not wait) for worlds without snapshots (test fakes)."""
    get_snapshot = getattr(world, "get_snapshot", None)
    return get_snapshot().frame if get_snapshot is not None else -1
