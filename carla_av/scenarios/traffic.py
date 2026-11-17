"""Background traffic around our car, driven by CARLA's Traffic Manager.

Adopted from Prithiv's Lanebot `traffic_node`: ordinary cars obey limits and lights,
some are slower than the limit, one is a slow lorry, one is a police car with blue
lights. Placed relative to our car at the start of the run, seeded so runs repeat.

The Traffic Manager runs in synchronous mode, so it advances with our world.tick().
Cars are handed back from the Traffic Manager before they are destroyed (deleting a car
the Traffic Manager still drives can make it abort - a lesson from Prithiv's node).
"""
from __future__ import annotations

import logging
import random
from typing import Optional

import carla

log = logging.getLogger(__name__)

TM_PORT = 8000
MIN_DISTANCE_M = 25.0         # keep spawn points this far from our car ...
MAX_DISTANCE_M = 180.0        # ... and no farther than this
CARS = ("vehicle.audi.a2", "vehicle.citroen.c3", "vehicle.mini.cooper_s", "vehicle.toyota.prius",
        "vehicle.nissan.micra", "vehicle.seat.leon", "vehicle.audi.tt", "vehicle.lincoln.mkz_2020")
LORRY = "vehicle.carlamotors.european_hgv"
POLICE = "vehicle.dodge.charger_police_2020"
SIRENS = (carla.VehicleLightState.Special1 | carla.VehicleLightState.LowBeam
          | carla.VehicleLightState.Position)


def traffic_manager(client: carla.Client, seed: int) -> carla.TrafficManager:
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(seed)
    tm.set_global_distance_to_leading_vehicle(4.0)
    return tm


def _blueprint(world: carla.World, name: str, role: str, rng: random.Random) -> Optional[carla.ActorBlueprint]:
    found = world.get_blueprint_library().filter(name)
    if not found:
        return None
    bp = found[0]
    bp.set_attribute("role_name", role)
    if bp.has_attribute("color"):
        bp.set_attribute("color", rng.choice(bp.get_attribute("color").recommended_values))
    return bp


def spawn_traffic(client: carla.Client, world: carla.World, ego: carla.Actor, count: int,
                  rng: random.Random, keep_clear: Optional[list[carla.Location]] = None) -> list[carla.Actor]:
    """Spawn `count` Traffic-Manager cars 25-180 m from our car. The first is a slow lorry,
    the second a police car with blue lights, the rest ordinary cars (0-40 % below the limit).

    `keep_clear` locations (scenario actors) get no traffic within 15 m."""
    if count <= 0:
        return []
    tm = traffic_manager(client, rng.randrange(1_000_000))
    here = ego.get_location()
    blocked = list(keep_clear or [])
    points = [p for p in world.get_map().get_spawn_points()
              if MIN_DISTANCE_M < p.location.distance(here) < MAX_DISTANCE_M
              and all(p.location.distance(b) > 15.0 for b in blocked)]
    rng.shuffle(points)
    spawned: list[carla.Actor] = []
    for point in points:
        if len(spawned) >= count:
            break
        kind = "lorry" if not spawned else "police" if len(spawned) == 1 else "car"
        name = {"lorry": LORRY, "police": POLICE}.get(kind) or rng.choice(CARS)
        bp = _blueprint(world, name, f"traffic_{kind}", rng)
        actor = world.try_spawn_actor(bp, point) if bp is not None else None
        if actor is None:
            continue
        actor.set_autopilot(True, TM_PORT)
        slower = {"lorry": 45.0, "police": -10.0}.get(kind, rng.uniform(0.0, 40.0))
        tm.vehicle_percentage_speed_difference(actor, slower)
        tm.distance_to_leading_vehicle(actor, 6.0)
        tm.random_left_lanechange_percentage(actor, 0)
        tm.random_right_lanechange_percentage(actor, 0)
        tm.ignore_lights_percentage(actor, 0)
        tm.ignore_signs_percentage(actor, 0)
        if kind == "police":
            tm.update_vehicle_lights(actor, False)
            actor.set_light_state(carla.VehicleLightState(SIRENS))
        else:
            tm.update_vehicle_lights(actor, True)
        spawned.append(actor)
    log.info("Traffic: %d vehicles (lorry, police, cars) around the ego car", len(spawned))
    return spawned


def release_traffic(client: carla.Client, actors: list[carla.Actor]) -> None:
    """Hand cars back from the Traffic Manager and stop its synchronous mode."""
    ids = [a.id for a in actors]
    try:
        if ids:
            client.apply_batch_sync([carla.command.SetAutopilot(i, False, TM_PORT) for i in ids], False)
        client.get_trafficmanager(TM_PORT).set_synchronous_mode(False)
    except RuntimeError as exc:
        log.warning("Could not release traffic: %s", exc)
