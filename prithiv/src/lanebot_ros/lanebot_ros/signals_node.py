"""Our car's lights, and spotting emergency vehicles.

  * Indicators follow the control node's state: left while moving out to
    overtake, right while moving back and while giving way.
  * Headlights are on low beam; at night we use high beam unless another car
    is in front of us or coming towards us, then we dip to low beam.
  * Rules on /lanebot/rules = [speed limit in m/s, metres to the next stop line,
    its light: 0 green or none, 1 amber, 2 red, next lower limit in m/s, metres to that sign]. Lights are looked up along our
    route up to 80 m ahead, so there is time to stop.
  * Emergency vehicles (ambulance, police, fire truck) on our side of the road
    are published on /lanebot/emergency as [distance, kind]:
    distance is negative behind us, positive in front; kind 1 = ambulance,
    2 = police, 3 = fire truck.
"""
import math

import carla
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from lanebot_ros.lane_node import choose

L = carla.VehicleLightState
DIP_RANGE = 200.0        # m, dip the high beam for cars this close in front of us
SAME_SIDE = 9.0          # m sideways: further than this is the other side of the road
KINDS = {'ambulance': 1.0, 'police': 2.0, 'firetruck': 3.0}
BLINKERS = {'SIGNAL_LEFT': 'left', 'CHANGE_LEFT': 'left',
            'SIGNAL_RIGHT': 'right', 'CHANGE_RIGHT': 'right', 'YIELD': 'right'}


class SignalsNode(Node):
    def __init__(self):
        super().__init__('signals_node')
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()
        self.map = self.world.get_map()
        self.stop_lines = self.find_stop_lines()
        self.state = 'CRUISE'
        self.last = None

        self.create_subscription(String, '/lanebot/state', lambda m: setattr(self, 'state', m.data), 10)
        self.emergency_pub = self.create_publisher(Float32MultiArray, '/lanebot/emergency', 10)
        self.lights_pub = self.create_publisher(String, '/lanebot/lights', 10)
        self.rules_pub = self.create_publisher(Float32MultiArray, '/lanebot/rules', 10)
        self.create_timer(0.1, self.tick)

    def find_stop_lines(self):
        """Every traffic light's stop line, by (road, lane)."""
        lines = {}
        for light in self.world.get_actors().filter('traffic.traffic_light'):
            for wp in light.get_stop_waypoints():
                lines.setdefault((wp.road_id, wp.lane_id), []).append((wp.transform.location, light))
        return lines

    def next_light(self, here):
        """Distance to the next stop line on our route, and its light (looks 80 m ahead)."""
        wp = self.map.get_waypoint(here)
        for step in range(40):
            for location, light in self.stop_lines.get((wp.road_id, wp.lane_id), []):
                if wp.transform.location.distance(location) < 2.5:
                    return max(0.0, here.distance(location)), light.get_state()
            options = wp.next(2.0)
            if not options:
                break
            wp = choose(wp, options)
        return float('inf'), carla.TrafficLightState.Green

    def next_speed_sign(self, here, current_kmh):
        """The nearest lower speed limit sign within 100 m ahead: (m/s, metres) or (inf, inf)."""
        wp = self.map.get_waypoint(here)
        best = (float('inf'), float('inf'))
        for sign in wp.get_landmarks_of_type(100.0, '274', False):
            if sign.value < current_kmh and sign.distance < best[1]:
                best = (sign.value / 3.6, sign.distance)
        return best

    def tick(self):
        vehicles = list(self.world.get_actors().filter('vehicle.*'))
        ego = next((v for v in vehicles if v.attributes.get('role_name') == 'ego_vehicle'), None)
        if ego is None:
            return
        t = ego.get_transform()
        fwd = t.get_forward_vector()
        right = t.get_right_vector()

        nearest_emergency, kind = float('inf'), 0.0
        car_in_front = None
        for v in vehicles:
            if v.id == ego.id:
                continue
            d = v.get_location() - t.location
            ahead = d.x * fwd.x + d.y * fwd.y          # metres in front of us
            side = d.x * right.x + d.y * right.y       # metres to our right
            k = next((val for key, val in KINDS.items() if key in v.type_id), 0.0)
            if k and abs(side) < SAME_SIDE and abs(ahead) < abs(nearest_emergency):
                nearest_emergency, kind = ahead, k
            if 0.0 < ahead < DIP_RANGE and abs(side) < 30.0:
                if car_in_front is None or ahead < car_in_front:
                    car_in_front = ahead

        self.emergency_pub.publish(Float32MultiArray(data=[nearest_emergency, kind]))

        # --- rules: the signed speed limit here, and the next traffic light on our route ---
        distance, light = self.next_light(t.location)
        code = {carla.TrafficLightState.Yellow: 1.0, carla.TrafficLightState.Red: 2.0}.get(light, 0.0)
        upcoming = self.next_speed_sign(t.location, ego.get_speed_limit())
        self.rules_pub.publish(Float32MultiArray(data=[ego.get_speed_limit() / 3.6, distance, code, *upcoming]))

        # --- lights ---
        night = self.world.get_weather().sun_altitude_angle < 0.0
        blinker = BLINKERS.get(self.state, 'none')
        beam = 'high' if night and car_in_front is None else 'low'
        lights = L.Position | L.LowBeam
        if beam == 'high':
            lights |= L.HighBeam
        if blinker == 'left':
            lights |= L.LeftBlinker
        elif blinker == 'right':
            lights |= L.RightBlinker
        ego.set_light_state(L(lights))
        self.lights_pub.publish(String(data=f'{blinker} {beam}'))

        now = (blinker, beam)
        if now != self.last:
            reason = ''
            if night and beam == 'low':
                reason = f' (car {car_in_front:.0f} m in front of us)'
            self.get_logger().info(f'indicator {blinker}, {beam} beam{reason}')
            self.last = now


def main():
    rclpy.init()
    node = SignalsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
