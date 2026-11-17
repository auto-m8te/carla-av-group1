"""Traffic around our car, driven by CARLA's Traffic Manager.

Ordinary cars obey the speed limits and traffic lights. Emergency vehicles use
their special rights (StVO §35/§38): blue lights and siren on, up to 30 % over
the limit, through red lights, changing lanes to get past.

Every cycle (90 s), placed relative to wherever our car is:
   0 s  three cars in the left lane coming up from behind to overtake us
  10 s  a slow car ahead in our lane (half the speed limit)
  30 s  three cars coming towards us on the other side
  45 s  a car ahead in our lane that brakes hard after 12 s
  60 s  an ambulance in our lane and a police car in the left lane, sirens on
  75 s  a slow lorry ahead in our lane
Cars far away or older than 150 s are removed.
"""
import math
import random
import time

import carla
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

TM_PORT = 8000
CYCLE = 90.0           # s
MAX_AGE = 150.0        # s
MAX_DISTANCE = 300.0   # m
LABELS = ('overtaker', 'slow_car', 'oncoming', 'braking_car', 'ambulance', 'police', 'lorry')
CARS = ['vehicle.audi.a2', 'vehicle.citroen.c3', 'vehicle.mini.cooper_s', 'vehicle.toyota.prius',
        'vehicle.nissan.micra', 'vehicle.seat.leon', 'vehicle.audi.tt', 'vehicle.mercedes.coupe_2020',
        'vehicle.lincoln.mkz_2020', 'vehicle.chevrolet.impala']
SIRENS = carla.VehicleLightState(carla.VehicleLightState.Special1 | carla.VehicleLightState.LowBeam
                                 | carla.VehicleLightState.Position)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class TrafficNode(Node):
    def __init__(self):
        super().__init__('traffic_node')
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.library = self.world.get_blueprint_library()
        self.tm = self.client.get_trafficmanager(TM_PORT)
        self.tm.set_global_distance_to_leading_vehicle(4.0)
        self.spawned = {}   # actor id -> (actor, time created)
        self.later = []     # (time, action) for things that happen after a delay
        self.to_delete = [] # cars handed back from the Traffic Manager, deleted next tick
        self.remove_leftovers()

        self.events = [(0.0, self.overtakers), (10.0, self.slow_car), (30.0, self.oncoming),
                       (45.0, self.braking_car), (60.0, self.emergency), (75.0, self.lorry)]
        self.cycle_start = time.monotonic()
        self.done = set()
        self.create_timer(0.5, self.tick)
        self.get_logger().info('Traffic started: a new set of vehicles every 90 s.')

    # ---------- finding places relative to our car ----------
    def ego(self):
        for a in self.world.get_actors().filter('vehicle.*'):
            if a.attributes.get('role_name') == 'ego_vehicle':
                return a
        return None

    @staticmethod
    def along(wp, distance):
        """Waypoint `distance` metres ahead (or behind, if negative) along OUR route:
        stay on the motorway (a lane to the left), then go straightest, as lane_node does."""
        def has_left(w):
            side = w.get_left_lane()
            return side is not None and side.lane_type == carla.LaneType.Driving and side.lane_id * w.lane_id > 0
        step = 2.0
        for _ in range(int(abs(distance) / step)):
            options = wp.next(step) if distance > 0 else wp.previous(step)
            if not options:
                return None
            yaw = wp.transform.rotation.yaw
            wp = min(options, key=lambda w: (not has_left(w), abs(wrap_deg(w.transform.rotation.yaw - yaw))))
        return wp

    @staticmethod
    def left_lane(wp):
        if wp is None:
            return None
        side = wp.get_left_lane()
        if side and side.lane_type == carla.LaneType.Driving and side.lane_id * wp.lane_id > 0:
            return side
        return None

    @staticmethod
    def oncoming_lane(wp):
        """The nearest lane going the other way."""
        lane = wp
        for _ in range(8):
            lane = lane.get_left_lane() if lane.lane_id * wp.lane_id > 0 else lane.get_right_lane()
            if lane is None:
                return None
            if lane.lane_type == carla.LaneType.Driving and lane.lane_id * wp.lane_id < 0:
                return lane
        return None

    # ---------- creating vehicles ----------
    def spawn(self, model, wp, label, slower_pct, emergency=False):
        """slower_pct: % below the speed limit (negative = above it, emergency vehicles only)."""
        if wp is None:
            return None
        bp = self.library.find(model)
        bp.set_attribute('role_name', label)
        if bp.has_attribute('color'):
            bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
        spot = wp.transform
        spot.location.z += 0.5
        car = self.world.try_spawn_actor(bp, spot)
        if car is None:
            self.get_logger().warn(f'Could not place the {label} (spot taken).')
            return None
        car.set_autopilot(True, TM_PORT)
        self.tm.vehicle_percentage_speed_difference(car, slower_pct)
        self.tm.distance_to_leading_vehicle(car, 6.0)
        self.tm.random_left_lanechange_percentage(car, 0)
        self.tm.random_right_lanechange_percentage(car, 0)
        if emergency:
            self.tm.auto_lane_change(car, True)
            self.tm.ignore_lights_percentage(car, 100)     # §35: may cross red lights
            self.tm.update_vehicle_lights(car, False)
            car.set_light_state(SIRENS)                     # blue lights on
        else:
            self.tm.auto_lane_change(car, label == 'overtaker')
            self.tm.ignore_lights_percentage(car, 0)
            self.tm.ignore_signs_percentage(car, 0)
            self.tm.update_vehicle_lights(car, True)        # indicators, headlights at night
        if label not in ('oncoming',):
            # Follow our motorway route instead of picking random turns (which took exits).
            self.tm.set_path(car, self.route_points(wp, 2000.0, 40.0))
        self.spawned[car.id] = (car, time.monotonic())
        return car

    def route_points(self, wp, length, every):
        """Locations every `every` metres along our route, for the Traffic Manager to follow."""
        points, travelled = [], 0.0
        while travelled < length:
            nxt = self.along(wp, every)
            if nxt is None:
                break
            wp, travelled = nxt, travelled + every
            points.append(wp.transform.location)
        return points

    def overtakers(self, ego_wp):
        left = self.left_lane(ego_wp)
        if left is None:
            return
        n = sum(self.spawn(random.choice(CARS), self.along(left, -gap), 'overtaker', 0) is not None
                for gap in (30.0, 60.0, 90.0))
        self.get_logger().info(f'{n} cars at the speed limit coming up behind us in the left lane.')

    def slow_car(self, ego_wp):
        if self.spawn(random.choice(CARS), self.along(ego_wp, 80.0), 'slow_car', 50):
            self.get_logger().info('A slow car (half the limit) 80 m ahead in our lane.')

    def oncoming(self, ego_wp):
        count = 0
        for gap in (150.0, 190.0, 230.0):
            ahead = self.along(ego_wp, gap)
            lane = self.oncoming_lane(ahead) if ahead else None
            count += self.spawn(random.choice(CARS), lane, 'oncoming', 10) is not None
        self.get_logger().info(f'{count} cars coming towards us on the other side.')

    def braking_car(self, ego_wp):
        car = self.spawn(random.choice(CARS), self.along(ego_wp, 60.0), 'braking_car', 10)
        if car:
            self.get_logger().info('A car 60 m ahead in our lane; it will brake hard in 12 s.')
            self.later.append((time.monotonic() + 12.0, lambda: self.brake(car)))

    def brake(self, car):
        if car.id in self.spawned:
            self.tm.vehicle_percentage_speed_difference(car, 75)
            self.get_logger().info('The car ahead brakes hard.')

    def emergency(self, ego_wp):
        a = self.spawn('vehicle.ford.ambulance', self.along(ego_wp, -100.0), 'ambulance', -30, emergency=True)
        left = self.left_lane(ego_wp)
        p = self.spawn('vehicle.dodge.charger_police_2020', self.along(left, -130.0) if left else None,
                       'police', -30, emergency=True)
        names = [n for n, car in (('ambulance', a), ('police car', p)) if car]
        if names:
            self.get_logger().info(f'{" and ".join(names).capitalize()} coming up behind us with blue lights and siren.')

    def lorry(self, ego_wp):
        if self.spawn('vehicle.carlamotors.european_hgv', self.along(ego_wp, 90.0), 'lorry', 45):
            self.get_logger().info('A slow lorry 90 m ahead in our lane.')

    # ---------- the schedule ----------
    def tick(self):
        ego = self.ego()
        if ego is None:
            return
        now = time.monotonic()
        t = now - self.cycle_start
        if t > CYCLE:
            self.cycle_start, self.done, t = now, set(), 0.0
        ego_wp = self.map.get_waypoint(ego.get_location())
        for when, event in self.events:
            if t >= when and when not in self.done:
                self.done.add(when)
                event(ego_wp)
        for item in [i for i in self.later if i[0] <= now]:
            self.later.remove(item)
            item[1]()

        # Tidy up: remove vehicles that are far away or old. Two steps: first take
        # them away from the Traffic Manager, then delete them half a second later
        # (deleting a car the Traffic Manager still drives makes it abort).
        if self.to_delete:
            self.client.apply_batch_sync([carla.command.DestroyActor(i) for i in self.to_delete], False)
            self.to_delete = []
        here = ego.get_location()
        alive = {a.id: a for a in self.world.get_actors().filter('vehicle.*')}
        old = [car_id for car_id, (_, born) in self.spawned.items()
               if car_id not in alive or alive[car_id].get_location().distance(here) > MAX_DISTANCE
               or now - born > MAX_AGE]
        self.release(old)

    def release(self, ids):
        """Hand cars back from the Traffic Manager; they are deleted on the next tick."""
        ids = [i for i in ids if i in self.spawned]
        for car_id in ids:
            self.spawned.pop(car_id)
        if ids:
            self.client.apply_batch_sync([carla.command.SetAutopilot(i, False, TM_PORT) for i in ids], False)
            self.to_delete.extend(ids)

    def remove_leftovers(self):
        """Vehicles we created in an earlier run that never got removed."""
        old = [a.id for a in self.world.get_actors().filter('vehicle.*')
               if a.attributes.get('role_name') in LABELS]
        if old:
            self.client.apply_batch_sync([carla.command.DestroyActor(i) for i in old], False)
            self.get_logger().info(f'Removed {len(old)} vehicles left over from an earlier run.')

    def remove_all(self):
        self.release(list(self.spawned))
        time.sleep(0.5)
        self.client.apply_batch_sync([carla.command.DestroyActor(i) for i in self.to_delete], False)
        self.to_delete = []


def main():
    rclpy.init()
    node = TrafficNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.remove_all()
    node.destroy_node()


if __name__ == '__main__':
    main()
