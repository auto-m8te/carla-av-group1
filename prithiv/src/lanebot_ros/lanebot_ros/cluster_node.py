"""Instrument cluster: a dashboard window showing what the car senses and decides.

Left:   speedometer with the signed speed limit.
Middle: top-down view: our car, the lane lines, and every lidar point at car
        height (red inside our lane ahead, amber in the left lane, grey elsewhere).
Right:  what the car is doing, indicators, headlights, distances, warnings.
"""
import math
import time

import numpy as np
import pygame
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Float32MultiArray, String

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
SCALE = 7.0            # pixels per metre in the top-down view
VIEW = pygame.Rect(400, 20, 380, 400)
STATES = {
    'CRUISE': ('Following the lane', TEXT),
    'SIGNAL_LEFT': ('Indicating to overtake', AMBER),
    'CHANGE_LEFT': ('Changing to the left lane', AMBER),
    'PASS': ('Overtaking', ACCENT),
    'SIGNAL_RIGHT': ('Indicating to move back', AMBER),
    'CHANGE_RIGHT': ('Moving back to the right', AMBER),
    'YIELD': ('Making way: emergency vehicle', BLUE),
}
INF = float('inf')


class ClusterNode(Node):
    def __init__(self):
        super().__init__('cluster_node')
        self.speed = 0.0
        self.limit = 0.0
        self.red_light = False
        self.light_distance = INF
        self.lane = [0.0, 0.0, 0.0, 0.0, 0.0, 3.5]
        self.lanes = [INF] * 4
        self.obstacle = INF
        self.state = 'CRUISE'
        self.blinker, self.beam = 'none', 'low'
        self.emergency = (INF, 0.0)
        self.points = np.zeros((0, 2), dtype=np.float32)

        self.create_subscription(Float32, '/carla/ego_vehicle/speedometer', lambda m: setattr(self, 'speed', m.data), 10)
        self.create_subscription(Float32MultiArray, '/lanebot/lane', lambda m: setattr(self, 'lane', list(m.data)), 10)
        self.create_subscription(Float32MultiArray, '/lanebot/lanes', lambda m: setattr(self, 'lanes', list(m.data)), 10)
        self.create_subscription(Float32, '/lanebot/obstacle_distance', lambda m: setattr(self, 'obstacle', m.data), 10)
        self.create_subscription(String, '/lanebot/state', lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(String, '/lanebot/lights', self.on_lights, 10)
        self.create_subscription(Float32MultiArray, '/lanebot/emergency', lambda m: setattr(self, 'emergency', tuple(m.data[:2])), 10)
        self.create_subscription(Float32MultiArray, '/lanebot/rules', self.on_rules, 10)
        self.create_subscription(PointCloud2, '/carla/ego_vehicle/lidar', self.on_lidar, 2)

        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption('Lanebot instrument cluster')
        self.font = {s: pygame.font.SysFont('dejavusans', s) for s in (14, 18, 24, 56)}
        self.bold = {s: pygame.font.SysFont('dejavusans', s, bold=True) for s in (18, 22, 30)}
        self.create_timer(1.0 / 20, self.draw)

    def on_lights(self, msg):
        parts = msg.data.split()
        if len(parts) == 2:
            self.blinker, self.beam = parts

    def on_rules(self, msg):
        self.limit = msg.data[0]
        self.red_light = int(msg.data[2]) > 0 and msg.data[1] < 80
        self.light_distance = msg.data[1]

    def on_lidar(self, msg):
        p = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[::3, :3]
        p = p[np.isfinite(p).all(axis=1)]
        keep = (p[:, 2] > -2.0) & (p[:, 2] < -0.4) & (np.abs(p[:, 0]) < 40) & (np.abs(p[:, 1]) < 25)
        self.points = p[keep, :2]

    # ---------- drawing helpers ----------
    def text(self, words, pos, size=18, colour=TEXT, bold=False, centre=False):
        img = (self.bold if bold else self.font)[size].render(words, True, colour)
        rect = img.get_rect(center=pos) if centre else img.get_rect(topleft=pos)
        self.screen.blit(img, rect)

    def to_screen(self, x, y):
        """Car frame (x forward, y left, metres) to pixels in the top-down view."""
        cx, cy = VIEW.centerx, VIEW.bottom - 120
        return int(cx - y * SCALE), int(cy - x * SCALE)

    def blink(self):
        return int(time.monotonic() / 0.38) % 2 == 0

    # ---------- the panels ----------
    def speedometer(self):
        cx, cy, r = 190, 215, 160
        pygame.draw.circle(self.screen, PANEL, (cx, cy), r + 12)
        top = 160.0
        for kmh in range(0, int(top) + 1, 20):
            a = math.radians(225 - 270 * kmh / top)
            x1, y1 = cx + r * math.cos(a), cy - r * math.sin(a)
            x2, y2 = cx + (r - 16) * math.cos(a), cy - (r - 16) * math.sin(a)
            pygame.draw.line(self.screen, QUIET, (x1, y1), (x2, y2), 3)
            self.text(str(kmh), (cx + (r - 36) * math.cos(a), cy - (r - 36) * math.sin(a)), 14, QUIET, centre=True)
        kmh = self.speed * 3.6
        if self.limit > 0:   # the limit as a red mark on the dial
            a = math.radians(225 - 270 * min(self.limit * 3.6, top) / top)
            pygame.draw.line(self.screen, RED, (cx + r * math.cos(a), cy - r * math.sin(a)),
                             (cx + (r - 24) * math.cos(a), cy - (r - 24) * math.sin(a)), 5)
        a = math.radians(225 - 270 * min(kmh, top) / top)
        pygame.draw.line(self.screen, ACCENT, (cx, cy), (cx + (r - 30) * math.cos(a), cy - (r - 30) * math.sin(a)), 5)
        pygame.draw.circle(self.screen, ACCENT, (cx, cy), 8)
        self.text(f'{kmh:.0f}', (cx, cy + 58), 56, TEXT, centre=True)
        self.text('km/h', (cx, cy + 98), 18, QUIET, centre=True)
        if self.limit > 0:   # German speed-limit sign
            pygame.draw.circle(self.screen, (245, 245, 245), (cx + 130, 70), 32)
            pygame.draw.circle(self.screen, RED, (cx + 130, 70), 32, 6)
            self.text(f'{self.limit * 3.6:.0f}', (cx + 130, 70), 24, (20, 20, 20), bold=False, centre=True)

    def top_view(self):
        pygame.draw.rect(self.screen, PANEL, VIEW, border_radius=12)
        clip = self.screen.get_clip()
        self.screen.set_clip(VIEW)
        offset, heading, curvature, left_shift, _, width = self.lane[:6]

        def lane_line(lateral, colour, dashed):
            pts = []
            for x in np.arange(-20, 42, 2.0):
                y = lateral - offset - heading * x + 0.5 * curvature * x * x
                pts.append(self.to_screen(x, y))
            for i in range(len(pts) - 1):
                if not dashed or i % 3 != 2:
                    pygame.draw.line(self.screen, colour, pts[i], pts[i + 1], 3)

        lane_line(-width / 2, (200, 200, 200), False)                    # right edge (solid)
        lane_line(width / 2, (200, 200, 200), left_shift > 0)          # our left line
        if left_shift > 0:
            lane_line(left_shift + width / 2, (200, 200, 200), False)

        for x, y in self.points:
            from_lane = y + heading * x - 0.5 * curvature * x * x + offset
            if x > 2.6 and abs(from_lane) < 1.0:
                colour = RED
            elif left_shift > 0 and abs(from_lane - left_shift) < 1.0:
                colour = AMBER
            else:
                colour = (110, 115, 125)
            pygame.draw.circle(self.screen, colour, self.to_screen(x, y), 2)

        # our car
        x0, y0 = self.to_screen(2.4, 0.95)
        x1, y1 = self.to_screen(-2.4, -0.95)
        pygame.draw.rect(self.screen, ACCENT, pygame.Rect(x0, y0, x1 - x0, y1 - y0), border_radius=6)
        if self.blinker != 'none' and self.blink():
            side = -1 if self.blinker == 'left' else 1
            for fx in (2.2, -2.2):
                pygame.draw.circle(self.screen, AMBER, self.to_screen(fx, -side * 0.95), 5)
        if math.isfinite(self.obstacle):
            ox, oy = self.to_screen(self.obstacle + 2.4, 0)
            pygame.draw.line(self.screen, RED, (ox - 30, oy), (ox + 30, oy), 3)
            self.text(f'{self.obstacle:.0f} m', (ox + 36, oy - 10), 14, RED)
        self.screen.set_clip(clip)

    def status(self):
        x = 810
        words, colour = STATES.get(self.state, (self.state, TEXT))
        self.text(words, (x, 26), 22, colour, bold=True)

        # indicators
        for side, cx in (('left', x + 20), ('right', x + 90)):
            on = self.blinker == side and self.blink()
            d = -1 if side == 'left' else 1
            arrow = [(cx + d * 22, 82), (cx, 66), (cx, 74), (cx - d * 18, 74), (cx - d * 18, 90), (cx, 90), (cx, 98)]
            pygame.draw.polygon(self.screen, GREEN if on else (45, 50, 58), arrow)
        # headlights
        beam_colour = BLUE if self.beam == 'high' else GREEN
        pygame.draw.circle(self.screen, beam_colour, (x + 170, 82), 12)
        self.text('high beam' if self.beam == 'high' else 'low beam', (x + 190, 72), 18, beam_colour)

        own_ahead, left_ahead, own_behind, left_behind = self.lanes[:4]
        rows = [('Obstacle ahead', self.obstacle), ('Our lane ahead', own_ahead), ('Our lane behind', own_behind),
                ('Left lane ahead', left_ahead), ('Left lane behind', left_behind)]
        for i, (name, d) in enumerate(rows):
            y = 126 + 28 * i
            self.text(name, (x, y), 18, QUIET)
            self.text('clear' if not math.isfinite(d) else f'{d:.0f} m', (x + 200, y), 18, TEXT)
        self.text('Lane offset', (x, 266), 18, QUIET)
        self.text(f'{self.lane[0]:+.2f} m', (x + 200, 266), 18, TEXT)

        distance, kind = self.emergency
        if kind and abs(distance) < 150:
            name = {1.0: 'Ambulance', 2.0: 'Police', 3.0: 'Fire engine'}.get(kind, 'Emergency vehicle')
            where = 'behind' if distance < 0 else 'ahead'
            colour = BLUE if self.blink() else RED
            box = pygame.Rect(x, 310, 340, 50)
            pygame.draw.rect(self.screen, colour, box, border_radius=10)
            self.text(f'{name} {abs(distance):.0f} m {where}', box.center, 22, (255, 255, 255), bold=True, centre=True)
        if self.red_light:
            box = pygame.Rect(x, 370, 340, 44)
            pygame.draw.rect(self.screen, RED, box, border_radius=10)
            self.text(f'Red light in {self.light_distance:.0f} m', box.center, 22, (255, 255, 255), bold=True, centre=True)

    def draw(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
        self.screen.fill(BG)
        self.speedometer()
        self.top_view()
        self.status()
        pygame.display.flip()


def main():
    rclpy.init()
    node = ClusterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    pygame.quit()
    node.destroy_node()


if __name__ == '__main__':
    main()
