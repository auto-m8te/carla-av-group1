"""Lane reference for the Gazebo track.

Publishes exactly what the CARLA lane node publishes, on the same topic, so
the obstacle and control nodes work unchanged:
    /lanebot/lane = [offset, heading_error, curvature, pass_shift]
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from lanebot_gz import track

WINDOW = 80   # stored points to search either side of the last match (20 m)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LaneNode(Node):
    def __init__(self):
        super().__init__('lane_node')
        self.index = None   # where on the track we were last time
        self.pub = self.create_publisher(Float32MultiArray, '/lanebot/lane', 10)
        self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.on_odom, 10)
        self.count = 0

    def nearest(self, x, y):
        """Index of the track point closest to the car."""
        if self.index is None:
            candidates = np.arange(len(track.S))            # first time: search everywhere
        else:
            candidates = (self.index + np.arange(-WINDOW, WINDOW + 1)) % len(track.S)
        d = np.hypot(track.XY[candidates, 0] - x, track.XY[candidates, 1] - y)
        return int(candidates[np.argmin(d)])

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.index = self.nearest(x, y)
        cx, cy, lane_yaw, curvature = track.pose_at(track.S[self.index])

        offset = -math.sin(lane_yaw) * (x - cx) + math.cos(lane_yaw) * (y - cy)
        heading_error = wrap(yaw - lane_yaw)
        pass_shift = track.LANE_WIDTH   # the overtaking lane is always 3.5 m to the left

        out = Float32MultiArray()
        out.data = [offset, heading_error, curvature, pass_shift]
        self.pub.publish(out)

        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(
                f'offset {offset:+.2f} m   heading error {math.degrees(heading_error):+.1f} deg')


def main():
    rclpy.init()
    node = LaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
