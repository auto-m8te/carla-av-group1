import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Float32MultiArray

MIN_X = 2.6         # m, ignore points on our own car (front)
REAR = 2.6          # m, ignore points on our own car (back)
MAX_X = 45.0        # m, how far ahead to look
MAX_BEHIND = 45.0   # m, how far behind to look (cars coming up to overtake)
HALF_WIDTH = 1.0    # m, half the width of the corridor we check in each lane
MIN_Z = -2.0        # m, lidar sits 2.4 m up, so this is 0.4 m above the road
MAX_Z = -0.4        # m, ignore things higher than 2 m (bridges, signs)
FRONT = 2.4         # m, from the lidar to our front bumper
MIN_POINTS = 3      # fewer points than this is treated as noise
SCANS = 3           # combine this many lidar scans
INF = float('inf')


class ObstacleNode(Node):
    """Finds what is around us in the lidar.

    Publishes two things:
      /lanebot/obstacle_distance  nearest thing straight ahead of the car
      /lanebot/lanes              [our lane ahead, left lane ahead,
                                   our lane behind, left lane behind]  in metres
    "Our lane" and "left lane" are measured from the lane the route follows,
    so they stay put while the car itself moves between lanes.
    """

    def __init__(self):
        super().__init__('obstacle_node')
        self.offset = 0.0
        self.heading_error = 0.0
        self.curvature = 0.0
        self.pass_shift = 0.0
        self.recent = []   # points kept from the last few scans
        self.count = 0

        self.create_subscription(Float32MultiArray, '/lanebot/lane', self.on_lane, 10)
        self.create_subscription(PointCloud2, '/carla/ego_vehicle/lidar', self.on_lidar, 10)
        self.pub = self.create_publisher(Float32, '/lanebot/obstacle_distance', 10)
        self.lanes_pub = self.create_publisher(Float32MultiArray, '/lanebot/lanes', 10)

    def on_lane(self, msg):
        self.offset, self.heading_error, self.curvature, self.pass_shift = msg.data[:4]

    def on_lidar(self, msg):
        # Each point is x (forward), y (left), z (up), intensity, ...
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        # Rays that hit nothing come back as infinity (in Gazebo); drop them.
        points = points[np.isfinite(points[:, :3]).all(axis=1)]
        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        # Keep points at car height that are not on our own car.
        keep = ((z > MIN_Z) & (z < MAX_Z) & (x > -MAX_BEHIND) & (x < MAX_X)
                & ~((x > -REAR) & (x < MIN_X) & (np.abs(y) < 1.2)))
        self.recent.append(np.stack([x[keep], y[keep]], axis=1))
        self.recent = self.recent[-SCANS:]
        pts = np.concatenate(self.recent)
        x, y = pts[:, 0], pts[:, 1]

        # Where the middle of "straight ahead along the lane" is, seen from the car.
        # It tilts if the car is not parallel to the lane, and bends with the road.
        centre = -self.heading_error * x + 0.5 * self.curvature * x * x

        # 1. Straight ahead of the car, wherever the car is.
        ahead = x[(x > MIN_X) & (np.abs(y - centre) < HALF_WIDTH)]
        out = Float32()
        out.data = float(np.sort(ahead)[MIN_POINTS - 1] - FRONT) if len(ahead) >= MIN_POINTS else INF
        self.pub.publish(out)

        # 2. Our lane and the left lane, measured from the route's lane centre.
        from_own_lane = y - centre + self.offset
        own = np.abs(from_own_lane) < HALF_WIDTH
        left = (np.abs(from_own_lane - self.pass_shift) < HALF_WIDTH) if self.pass_shift else np.zeros_like(own)
        lanes = Float32MultiArray()
        lanes.data = [self.nearest(x[own & (x > MIN_X)]) - FRONT,
                      self.nearest(x[left & (x > 0.0)]),
                      # includes cars alongside us (negative), so we never move back onto one
                      self.nearest(-x[own & (x < MIN_X)]) - FRONT,
                      self.nearest(-x[left & (x <= 0.0)])]
        self.lanes_pub.publish(lanes)

        self.count += 1
        if self.count % 10 == 0 and out.data != INF:
            self.get_logger().info(f'obstacle {out.data:.1f} m ahead')

    @staticmethod
    def nearest(distances):
        """Distance to the nearest thing, or infinity if there is nothing."""
        if len(distances) < MIN_POINTS:
            return INF
        return float(np.sort(distances)[MIN_POINTS - 1])


def main():
    rclpy.init()
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
