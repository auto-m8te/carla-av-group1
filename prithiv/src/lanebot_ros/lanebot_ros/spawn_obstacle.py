import math
import sys

import carla


def main():
    # How far ahead of the car to put the obstacle, in metres.
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 80.0

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    world.wait_for_tick()

    # Find our car by the name the bridge gave it.
    cars = [a for a in world.get_actors().filter('vehicle.*')
            if a.attributes.get('role_name') == 'ego_vehicle']
    if not cars:
        print('No ego vehicle found. Is the simulation running?')
        return
    wp = world.get_map().get_waypoint(cars[0].get_location())

    # Walk forward along the car's lane.
    travelled = 0.0
    while travelled < distance:
        options = wp.next(2.0)
        yaw = wp.transform.rotation.yaw
        wp = min(options, key=lambda w: abs(
            math.atan2(math.sin(math.radians(w.transform.rotation.yaw - yaw)),
                       math.cos(math.radians(w.transform.rotation.yaw - yaw)))))
        travelled += 2.0

    # Park a car there with the handbrake on.
    blueprint = world.get_blueprint_library().find('vehicle.audi.tt')
    blueprint.set_attribute('role_name', 'obstacle')
    spot = wp.transform
    spot.location.z += 0.5
    obstacle = world.spawn_actor(blueprint, spot)
    obstacle.apply_control(carla.VehicleControl(hand_brake=True))
    print(f'Obstacle parked {distance:.0f} m ahead of the car.')


if __name__ == '__main__':
    main()
