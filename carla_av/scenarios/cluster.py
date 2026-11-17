"""Instrument cluster: a dashboard window showing what the car senses and decides.

Ported from Prithiv's Lanebot `cluster_node` (ROS 2 + pygame). Instead of subscribing
to ROS topics, the scenario runner hands it a `ClusterState` every tick.

Left:   speedometer with the speed limit.
Middle: top-down view: our car, the lane lines and the LiDAR points at car height
        (red inside our lane ahead, amber in the overtaking lane, grey elsewhere).
Right:  what the car is doing, indicators, distances, warnings (red light,
        emergency vehicle, pedestrian).
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

W, H = 1180, 440
BG = (12, 14, 18)
PANEL = (24, 27, 33)
TEXT = (230, 232, 236)
QUIET = (130, 136, 148)
ACCENT = (62, 140, 255)
GREEN = (60, 200, 110)
AMBER = (245, 175, 40)
RED = (235, 70, 60)
BLUE = (70, 120, 255)
SCALE = 10.0                # pixels per metre in the top-down view
BLINK_PERIOD = 0.38         # s
POINT_Z = (0.3, 2.0)        # LiDAR points between these heights (vehicle frame) are drawn
STATES = {
    "CRUISE": ("Following the lane", TEXT),
    "FOLLOW": ("Following the car ahead", AMBER),
    "BRAKE": ("Braking", AMBER),
    "STOPPED": ("Stopped", RED),
    "EMERGENCY_STOP": ("Emergency stop", RED),
    "AVOID": ("Changing lane to overtake", AMBER),
    "PASSING": ("Overtaking", ACCENT),
    "RETURN": ("Moving back to our lane", AMBER),
    "ABORT": ("Aborting: back behind", RED),
    "YIELD": ("Making way: emergency vehicle", BLUE),
}


def _keep_on_top(pygame) -> None:
    """Keep the dashboard above the CARLA window (Windows), so it does not open hidden
    behind the simulator."""
    if sys.platform != "win32":
        return
    try:
        import ctypes
        hwnd = pygame.display.get_wm_info()["window"]
        topmost, no_move_size = -1, 0x0001 | 0x0002
        ctypes.windll.user32.SetWindowPos(hwnd, topmost, 0, 0, 0, 0, no_move_size)
    except (AttributeError, KeyError, OSError):
        pass


@dataclass
class ClusterState:
    """Everything the dashboard shows, in our conventions (+ lateral = right)."""
    speed: float = 0.0                      # m/s
    limit: Optional[float] = None           # m/s
    mode: str = "CRUISE"
    blinker: Optional[str] = None           # "left" / "right" / None
    obstacle: Optional[float] = None        # m, bumper gap ahead
    lateral_offset: float = 0.0             # m, + = right of lane centre
    heading_error: float = 0.0              # rad, + = pointing right
    curvature: float = 0.0                  # 1/m, + = turning right
    lane_width: float = 3.5
    overtake_side: int = -1                 # -1 = left lane, +1 = right lane
    adjacent_clear: bool = False
    light_gap: Optional[float] = None       # m to a red/amber stop line
    emergency_distance: Optional[float] = None
    emergency_kind: Optional[str] = None
    pedestrian_near: bool = False
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))  # vehicle frame x fwd, y right


class InstrumentCluster:
    def __init__(self, title: str = "Group 1 instrument cluster") -> None:
        import pygame  # local import: pygame is only needed when the dashboard is shown
        self.pg = pygame
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption(title)
        _keep_on_top(pygame)
        self.view = pygame.Rect(400, 20, 380, 400)
        self.font = {s: pygame.font.SysFont("segoeui,arial", s) for s in (14, 18, 24, 56)}
        self.bold = {s: pygame.font.SysFont("segoeui,arial", s, bold=True) for s in (18, 22, 30)}
        self.closed = False

    # ---------- helpers ----------
    def text(self, words: str, pos, size=18, colour=TEXT, bold=False, centre=False) -> None:
        img = (self.bold if bold else self.font)[size].render(words, True, colour)
        rect = img.get_rect(center=pos) if centre else img.get_rect(topleft=pos)
        self.screen.blit(img, rect)

    def to_screen(self, x: float, y_left: float) -> tuple[int, int]:
        """Car frame (x forward, y LEFT, metres) to pixels in the top-down view."""
        cx, cy = self.view.centerx, self.view.bottom - 120
        return int(cx - y_left * SCALE), int(cy - x * SCALE)

    @staticmethod
    def blink() -> bool:
        return int(time.monotonic() / BLINK_PERIOD) % 2 == 0

    # ---------- panels ----------
    def speedometer(self, st: ClusterState) -> None:
        pg, cx, cy, r, top = self.pg, 190, 215, 160, 80.0
        pg.draw.circle(self.screen, PANEL, (cx, cy), r + 12)
        for kmh in range(0, int(top) + 1, 10):
            a = math.radians(225 - 270 * kmh / top)
            pg.draw.line(self.screen, QUIET, (cx + r * math.cos(a), cy - r * math.sin(a)),
                         (cx + (r - 16) * math.cos(a), cy - (r - 16) * math.sin(a)), 3)
            self.text(str(kmh), (cx + (r - 36) * math.cos(a), cy - (r - 36) * math.sin(a)), 14, QUIET, centre=True)
        if st.limit:
            a = math.radians(225 - 270 * min(st.limit * 3.6, top) / top)
            pg.draw.line(self.screen, RED, (cx + r * math.cos(a), cy - r * math.sin(a)),
                         (cx + (r - 24) * math.cos(a), cy - (r - 24) * math.sin(a)), 5)
            pg.draw.circle(self.screen, (245, 245, 245), (cx + 130, 70), 32)
            pg.draw.circle(self.screen, RED, (cx + 130, 70), 32, 6)
            self.text(f"{st.limit * 3.6:.0f}", (cx + 130, 70), 24, (20, 20, 20), centre=True)
        kmh = st.speed * 3.6
        a = math.radians(225 - 270 * min(kmh, top) / top)
        pg.draw.line(self.screen, ACCENT, (cx, cy), (cx + (r - 30) * math.cos(a), cy - (r - 30) * math.sin(a)), 5)
        pg.draw.circle(self.screen, ACCENT, (cx, cy), 8)
        self.text(f"{kmh:.0f}", (cx, cy + 58), 56, TEXT, centre=True)
        self.text("km/h", (cx, cy + 98), 18, QUIET, centre=True)

    def top_view(self, st: ClusterState) -> None:
        pg = self.pg
        pg.draw.rect(self.screen, PANEL, self.view, border_radius=12)
        clip = self.screen.get_clip()
        self.screen.set_clip(self.view)
        # Prithiv's drawing uses y LEFT; our values are + right, so flip the signs.
        offset, heading, curvature = -st.lateral_offset, -st.heading_error, -st.curvature
        width = st.lane_width
        other = -st.overtake_side * width          # centre of the overtaking lane, y-left

        def centre(x: float) -> float:
            return -heading * x + 0.5 * curvature * x * x

        def lane_line(lateral: float, dashed: bool) -> None:
            pts = [self.to_screen(x, lateral - offset + centre(x)) for x in np.arange(-20, 42, 2.0)]
            for i in range(len(pts) - 1):
                if not dashed or i % 3 != 2:
                    pg.draw.line(self.screen, (200, 200, 200), pts[i], pts[i + 1], 3)

        lane_line(-width / 2, st.overtake_side > 0)
        lane_line(width / 2, st.overtake_side < 0)
        lane_line(other + math.copysign(width / 2, other), False)

        pts = st.points
        if len(pts):
            pts = pts[(pts[:, 2] > POINT_Z[0]) & (pts[:, 2] < POINT_Z[1])
                      & (np.abs(pts[:, 0]) < 40) & (np.abs(pts[:, 1]) < 25)][::2]
        for x, y_right, _ in pts:
            y = -y_right
            from_lane = y - centre(x) + offset
            if x > 2.6 and abs(from_lane) < 1.0:
                colour = RED
            elif abs(from_lane - other) < 1.0:
                colour = AMBER
            else:
                colour = (110, 115, 125)
            pg.draw.circle(self.screen, colour, self.to_screen(x, y), 2)

        x0, y0 = self.to_screen(2.4, 0.95)
        x1, y1 = self.to_screen(-2.4, -0.95)
        pg.draw.rect(self.screen, ACCENT, pg.Rect(x0, y0, x1 - x0, y1 - y0), border_radius=6)
        if st.blinker and self.blink():
            side = 1 if st.blinker == "left" else -1
            for fx in (2.2, -2.2):
                pg.draw.circle(self.screen, AMBER, self.to_screen(fx, side * 0.95), 5)
        if st.obstacle is not None:
            ox, oy = self.to_screen(st.obstacle + 2.4, 0)
            pg.draw.line(self.screen, RED, (ox - 30, oy), (ox + 30, oy), 3)
            self.text(f"{st.obstacle:.0f} m", (ox + 36, oy - 10), 14, RED)
        self.screen.set_clip(clip)

    def status(self, st: ClusterState) -> None:
        pg, x = self.pg, 810
        words, colour = STATES.get(st.mode, (st.mode, TEXT))
        self.text(words, (x, 26), 22, colour, bold=True)
        for side, cx in (("left", x + 20), ("right", x + 90)):
            on = st.blinker == side and self.blink()
            d = -1 if side == "left" else 1
            arrow = [(cx + d * 22, 82), (cx, 66), (cx, 74), (cx - d * 18, 74), (cx - d * 18, 90), (cx, 90), (cx, 98)]
            pg.draw.polygon(self.screen, GREEN if on else (45, 50, 58), arrow)
        rows = [("Obstacle ahead", "clear" if st.obstacle is None else f"{st.obstacle:.0f} m"),
                ("Overtaking lane", "free" if st.adjacent_clear else "blocked"),
                ("Lane offset", f"{st.lateral_offset:+.2f} m"),
                ("Speed limit", "-" if not st.limit else f"{st.limit * 3.6:.0f} km/h")]
        for i, (name, value) in enumerate(rows):
            y = 126 + 28 * i
            self.text(name, (x, y), 18, QUIET)
            self.text(value, (x + 200, y), 18, TEXT)
        banners = []
        d = st.emergency_distance
        if st.emergency_kind and d is not None and abs(d) < 150:
            where = "behind" if d < 0 else "ahead"
            banners.append((f"{st.emergency_kind} {abs(d):.0f} m {where}", BLUE if self.blink() else RED))
        if st.light_gap is not None:
            banners.append((f"Red light in {st.light_gap:.0f} m", RED))
        if st.pedestrian_near:
            banners.append(("Pedestrian ahead", AMBER))
        for i, (words, colour) in enumerate(banners[:3]):
            box = pg.Rect(x, 250 + 56 * i, 340, 46)
            pg.draw.rect(self.screen, colour, box, border_radius=10)
            self.text(words, box.center, 22, (255, 255, 255), bold=True, centre=True)

    def update(self, st: ClusterState) -> None:
        if self.closed:
            return
        for event in self.pg.event.get():
            if event.type == self.pg.QUIT:   # closing the window just hides the dashboard
                self.close()
                return
        self.screen.fill(BG)
        self.speedometer(st)
        self.top_view(st)
        self.status(st)
        self.pg.display.flip()

    def close(self) -> None:
        if not self.closed:
            self.pg.quit()
            self.closed = True
