"""Park a car-sized box in our lane, a given distance ahead of the car."""
import subprocess
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry

from lanebot_gz import track

BOX = """<sdf version="1.9"><model name="{name}"><static>true</static><link name="l">
<collision name="c"><pose>0 0 0.75 0 0 0</pose><geometry><box><size>4.5 1.9 1.5</size></box></geometry></collision>
<visual name="v"><pose>0 0 0.75 0 0 0</pose><geometry><box><size>4.5 1.9 1.5</size></box></geometry>
<material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual>
</link></model></sdf>"""


def car_position():
    """Wait for one odometry message and return the car's x, y."""
    rclpy.init()
    node = rclpy.create_node('spawn_obstacle')
    got = []
    node.create_subscription(Odometry, '/carla/ego_vehicle/odometry',
                             lambda m: got.append((m.pose.pose.position.x, m.pose.pose.position.y)), 1)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if got:
            break
    node.destroy_node()
    rclpy.shutdown()
    return got[0] if got else None


def main():
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 80.0
    pos = car_position()
    if pos is None:
        print('No odometry from the car. Is the Gazebo simulation running?')
        return
    here = track.S[np.argmin(np.hypot(track.XY[:, 0] - pos[0], track.XY[:, 1] - pos[1]))]
    x, y, yaw, _ = track.pose_at(here + distance)
    name = f'obstacle_{int(here + distance)}'
    subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'lanebot', '-name', name,
                    '-string', BOX.format(name=name), '-x', f'{x:.2f}', '-y', f'{y:.2f}', '-z', '0',
                    '-Y', f'{yaw:.3f}'], check=False, capture_output=True)
    print(f'Obstacle parked {distance:.0f} m ahead of the car.')


if __name__ == '__main__':
    main()
