"""Capture camera frames + ground-truth lane offset/heading for tuning the lane detector.

Drives the ego with the map-waypoint steering plus a slow sinusoidal weave, so frames
cover lateral offsets of roughly ±1 m. Every N ticks it saves the raw RGB image and a
JSON line with the true lateral offset (m, + = right of lane centre) and heading error
(rad, + = pointing right), computed from the CARLA map.

Usage (CARLA running):  python -m tools.capture_lane_dataset --town Town01 --out ../data/lane
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import queue
import random
from pathlib import Path

import carla
import numpy as np

from avoidance.carla_adapter import waypoint_lane_steer

log = logging.getLogger("capture")
DT = 0.05
TARGET_SPEED = 8.0
WEAVE_AMPLITUDE = 0.12   # steer added on top of lane following
WEAVE_PERIOD_S = 6.0


def _gt_lane_pose(cmap: carla.Map, vehicle: carla.Vehicle) -> tuple[float, float]:
    tf = vehicle.get_transform()
    wp = cmap.get_waypoint(tf.location)
    d = tf.location - wp.transform.location
    right = wp.transform.get_right_vector()
    offset = d.x * right.x + d.y * right.y
    diff = math.radians(tf.rotation.yaw - wp.transform.rotation.yaw)
    return offset, math.atan2(math.sin(diff), math.cos(diff))


def _spawn(world: carla.World, rng: random.Random) -> tuple[carla.Vehicle, carla.Actor, "queue.Queue"]:
    bp_lib = world.get_blueprint_library()
    points = world.get_map().get_spawn_points()
    rng.shuffle(points)
    ego = next(v for v in (world.try_spawn_actor(bp_lib.find("vehicle.tesla.model3"), p) for p in points) if v)
    cam_bp = bp_lib.find("sensor.camera.rgb")
    for key, value in (("image_size_x", 800), ("image_size_y", 450), ("fov", 90)):
        cam_bp.set_attribute(key, str(value))
    cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=1.5, z=1.6), carla.Rotation(pitch=-8)),
                            attach_to=ego)
    images: "queue.Queue" = queue.Queue()
    cam.listen(images.put)
    return ego, cam, images


def _drive(world, ego, steer_fn, tick: int) -> None:
    v = ego.get_velocity()
    speed = math.hypot(v.x, v.y)
    throttle = max(0.0, min(0.6, 0.35 + 0.15 * (TARGET_SPEED - speed)))
    weave = WEAVE_AMPLITUDE * math.sin(2 * math.pi * tick * DT / WEAVE_PERIOD_S)
    ego.apply_control(carla.VehicleControl(throttle=throttle, steer=max(-1.0, min(1.0, steer_fn(ego) + weave))))


def capture(client: carla.Client, town: str, out: Path, drives: int, ticks: int, every: int, seed: int) -> int:
    import cv2  # local import: only this tool needs it at module level

    world = client.get_world()
    if not world.get_map().name.endswith(town):
        world = client.load_world(town)
    settings = world.get_settings()
    settings.synchronous_mode, settings.fixed_delta_seconds = True, DT
    world.apply_settings(settings)
    rng, saved = random.Random(seed), 0
    out.mkdir(parents=True, exist_ok=True)
    try:
        with (out / "labels.jsonl").open("a") as labels:
            for drive in range(drives):
                ego, cam, images = _spawn(world, rng)
                steer_fn = waypoint_lane_steer(world)
                try:
                    for tick in range(ticks):
                        _drive(world, ego, steer_fn, tick)
                        world.tick()
                        image = images.get(timeout=5.0)
                        if tick < 40 or tick % every:
                            continue
                        offset, heading = _gt_lane_pose(world.get_map(), ego)
                        bgr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)[:, :, :3]
                        name = f"{town}_{drive}_{tick:05d}.png"
                        cv2.imwrite(str(out / name), bgr)
                        labels.write(json.dumps({"file": name, "town": town, "offset_m": offset,
                                                 "heading_rad": heading}) + "\n")
                        saved += 1
                finally:
                    cam.stop()
                    cam.destroy()
                    ego.destroy()
    finally:
        settings.synchronous_mode, settings.fixed_delta_seconds = False, None
        world.apply_settings(settings)
    return saved


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--town", required=True)
    ap.add_argument("--out", default="../data/lane")
    ap.add_argument("--drives", type=int, default=3)
    ap.add_argument("--ticks", type=int, default=600)
    ap.add_argument("--every", type=int, default=6)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    client = carla.Client("localhost", 2000)
    client.set_timeout(180.0)
    n = capture(client, args.town, Path(args.out), args.drives, args.ticks, args.every, args.seed)
    log.info("saved %d labelled frames for %s", n, args.town)


if __name__ == "__main__":
    main()
