"""Chase-camera MP4 recording; rendering helpers also work without CARLA."""
from pathlib import Path
import math
import queue

import cv2
import numpy as np

from perception.sensor_sync import current_frame, wait_for_frame
from scenarios.hud_colors import HUD_COLORS
from avoidance.collision_avoidance import Mode

SCENARIO_TITLES = {
    "empty_lane": "Clear road -> follow the lane",
    "obstacle_stop": "Parked car, adjacent lane blocked -> stop",
    "obstacle_avoid": "Parked car, adjacent lane free -> change lane, pass, return",
    "curve_lane": "Sharp curve -> follow the lane",
    "curve_stop": "Parked car at curve apex, adjacent blocked -> stop",
    "lead_vehicle": "Slower lead vehicle -> follow at a safe distance",
    "lead_brake": "Lead vehicle brakes, adjacent blocked -> stop safely",
    "oncoming_overtake": "Oncoming traffic -> wait, then overtake when clear",
}


def _text(frame, text, xy, scale=0.65, color=(255, 255, 255), thickness=1):
    cv2.putText(frame, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                thickness, cv2.LINE_AA)


def draw_hud(frame, mode, speed, gap, extra):
    """Draw in place on a BGR image. Speed is supplied in m/s."""
    h, w = frame.shape[:2]
    scale = min(w / 1280, h / 720)
    panel = frame.copy()
    cv2.rectangle(panel, (0, 0), (min(w, int(650 * scale)), int(225 * scale)), (15, 15, 15), -1)
    cv2.addWeighted(panel, 0.8, frame, 0.2, 0, dst=frame)
    def line(text, y, size=0.7, color=(255, 255, 255), thickness=1):
        _text(frame, text, (int(20 * scale), int(y * scale)), size * scale, color, thickness)
    line(f"{extra.get('scenario', '')} | {extra.get('town', '')} | Run {extra.get('run', '')}", 32)
    mode = Mode(mode)
    line(mode.value, 78, 1.2, HUD_COLORS[mode][::-1], 2)  # CARLA RGB -> OpenCV BGR
    line(f"Speed: {speed * 3.6:.1f} km/h", 115)
    gap_text = "clear" if gap is None or not math.isfinite(gap) else f"{gap:.1f} m"
    line(f"Obstacle gap: {gap_text}", 148)
    line(f"Lane: {extra.get('lane_source', 'groundtruth')}", 180)
    line(f"Obstacle: {extra.get('obstacle_source', 'groundtruth')}", 212)
    # Hershey fonts do not support the middle dot; render it as a small circle.
    footer = "Group 1   SRH Berlin   CARLA 0.9.16"
    font_scale = 0.48 * scale
    (tw, _), _ = cv2.getTextSize(footer, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
    x, y = max(0, w - tw - int(18 * scale)), h - max(4, int(18 * scale))
    _text(frame, footer, (x, y), font_scale)
    for prefix in ("Group 1 ", "Group 1   SRH Berlin "):
        (offset, _), _ = cv2.getTextSize(prefix, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
        cv2.circle(frame, (x + offset, y - max(1, int(3 * scale))), max(1, int(scale)), (255, 255, 255), -1)
    return frame


def title_card(size, extra):
    w, h = size
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    scenario = extra.get("scenario", "")
    for text, y, scale in ((scenario.replace('_', ' ').title(), h * .42, 1.4),
                           (SCENARIO_TITLES.get(scenario, scenario), h * .53, .7),
                           (f"{extra.get('town', '')} | Run {extra.get('run', '')}", h * .64, .75)):
        scale = min(scale * w / 1280, (w - 30) / max(1, cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0][0]))
        width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
        _text(frame, text, ((w - width) // 2, int(y)), scale)
    return frame


class VideoRecorder:
    def __init__(self, world, ego, path, fps=20, size=(1280, 720)):
        import carla
        if fps <= 0 or min(size) <= 0:
            raise ValueError("fps and image dimensions must be positive")
        self.world, self.fps, self.size = world, fps, size
        self._closed, self._started = False, False
        self._queue = queue.Queue()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not self._writer.isOpened():
            self._writer.release()
            raise RuntimeError(f"Could not open video writer: {path}")
        try:
            bp = world.get_blueprint_library().find("sensor.camera.rgb")
            for key, value in (("image_size_x", size[0]), ("image_size_y", size[1]),
                               ("fov", 90), ("sensor_tick", 0.0)):
                bp.set_attribute(key, str(value))
            self.sensor = world.spawn_actor(bp, carla.Transform(
                carla.Location(x=-7.5, z=3.2), carla.Rotation(pitch=-12)), attach_to=ego)
            try:
                self.sensor.listen(self._queue.put)
            except Exception:
                self.sensor.destroy()
                raise
        except Exception:
            self.close()
            raise

    def add_frame(self, mode, speed, gap, extra: dict):
        if self._closed:
            return
        if not self._started:
            card = title_card(self.size, extra)
            for _ in range(round(2 * self.fps)):
                self._writer.write(card)
            self._started = True
        measurement = wait_for_frame(self._queue, current_frame(self.world))
        if measurement is None:
            raise RuntimeError("No chase-camera frame received")
        frame = np.frombuffer(measurement.raw_data, dtype=np.uint8).reshape(
            measurement.height, measurement.width, 4)[:, :, :3].copy()
        self._writer.write(draw_hud(frame, mode, speed, gap, extra))

    def close(self):
        """Release the writer once. Sensor destruction belongs to batch teardown."""
        if not self._closed:
            self._closed = True
            self._writer.release()
