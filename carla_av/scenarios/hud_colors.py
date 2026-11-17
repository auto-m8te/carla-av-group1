"""Shared controller HUD colours (RGB, as used by CARLA)."""
from avoidance.collision_avoidance import Mode

HUD_COLORS = {
    Mode.CRUISE: (0, 200, 0),
    Mode.FOLLOW: (220, 200, 0),
    Mode.BRAKE: (255, 140, 0),
    Mode.STOPPED: (220, 0, 0),
    Mode.EMERGENCY_STOP: (255, 0, 255),
    Mode.ABORT: (255, 170, 0),
    Mode.AVOID: (0, 120, 255),
    Mode.PASSING: (0, 120, 255),
    Mode.RETURN: (0, 220, 220),
    Mode.YIELD: (60, 110, 255),
}
